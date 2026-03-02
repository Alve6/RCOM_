// Implementação da camada de enlace (Link Layer)
//
// Este ficheiro implementa as funções básicas do protocolo de enlace
// usado no laboratório: estabelecimento de ligação (SET/UA), envio de
// I-frames com BCC2 e byte-stuffing, e receção com verificação de BCC2.
// Comentários adicionais em Português estão espalhados pelas funções
// principais para ajudar na compreensão.

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256
/* signal-safe alarm flag */
volatile sig_atomic_t alarmFired = 0;
int alarmCount = 0;
// link-layer runtime state (set by llopen)
static int link_timeout = 0;
static int link_retrans = 0;
static int link_opened = 0;
static int seq_tx = 0; // transmit sequence (0 or 1)
static int seq_rx = 0; // receive sequence (starts at 0 to match TX)

// Protocol constants
#define FLAG 0x7E
#define ESC 0x7D
#define A_ER 0x03  // Address Emitter 
#define A_RE 0x01  // Address Receiver
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_N(n) ((n) ? 0x40 : 0x00)  // Control I-frame with sequence n
#define C_RR(n) ((n) ? 0x85 : 0x05) // Control RR with sequence n
#define C_REJ(n) ((n) ? 0x81 : 0x01) // Control REJ with sequence n

typedef enum
{
    S_START,
    S_FLAG,
    S_A,
    S_C,
    S_BCC,
    S_STOPP
} State;

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    READING_DATA,
    DATA_FOUND_ESC,
    STOP_R
} LinkLayerState;

// Alarm function handler.
// This function will run whenever the signal SIGALRM is received.
void alarmHandler(int signal)
{
    (void)signal;
    alarmFired = 1;
    alarmCount++;
}

// Helper function to send supervision frames (RR/REJ/DISC/UA)
int sendSupervisionFrame(unsigned char address, unsigned char control)
{
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;
    frame[4] = FLAG;
    
    return writeBytesSerialPort(frame, 5);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    unsigned char byte;

    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    // Aqui abrimos a porta série com os parâmetros fornecidos em
    // `connectionParameters`. Se a abertura falhar retornamos -1.
    // Guardamos também `timeout` e `nRetransmissions` em variáveis
    // por funções como `llwrite`/`llread`.

    // save runtime params for llwrite/llread
    link_timeout = connectionParameters.timeout;
    link_retrans = connectionParameters.nRetransmissions;
    link_opened = 1;
    seq_tx = 0;
    seq_rx = 0;

    const unsigned char A_SET = 0x03;
    const unsigned char A_UA = 0x01;

    // Receiver: wait for SET, send UA
    if (connectionParameters.role == LlRx)
    {
    // Receptor: aguardamos um quadro SET enviado pelo transmissor.
    // O quadro SET tem esta forma: FLAG A_SET C_SET BCC1 FLAG
    // onde BCC1 == A_SET ^ C_SET. Se recebermos um SET correcto,
    // respondemos com um quadro UA para indicar que a ligação está
    // estabelecida.
    // Este código faz a leitura byte-a-byte e implementa uma
    // pequena máquina de estados para identificar o SET.
    State state = S_START;
        int done = 0;

        while (!done)
        {
            int rv = readByteSerialPort(&byte);
            if (rv < 0)
            {
                perror("readByteSerialPort");
                closeSerialPort();
                return -1;
            }
            if (rv == 0)
                continue;

            switch (state)
            {
            case S_START:
                if (byte == FLAG)
                    state = S_FLAG;
                break;
            case S_FLAG:
                if (byte == A_SET)
                    state = S_A;
                else if (byte == FLAG)
                    state = S_FLAG;
                else
                    state = S_START;
                break;
            case S_A:
                if (byte == C_SET)
                    state = S_C;
                else if (byte == FLAG)
                    state = S_FLAG;
                else
                    state = S_START;
                break;
            case S_C:
                if (byte == (A_SET ^ C_SET))
                    state = S_BCC;
                else if (byte == FLAG)
                    state = S_FLAG;
                else
                    state = S_START;
                break;
            case S_BCC:
                if (byte == FLAG)
                    done = 1;
                else
                    state = S_START;
                break;
            default:
                state = S_START;
                break;
            }
        }

        unsigned char ua[5];
        ua[0] = FLAG;
        ua[1] = A_UA;
        ua[2] = C_UA;
        ua[3] = A_UA ^ C_UA;
        ua[4] = FLAG;

        // Enviamos UA (Unnumbered Acknowledgement). Formato: FLAG A_UA C_UA BCC1 FLAG
        // BCC1 é A_UA ^ C_UA.
        if (writeBytesSerialPort(ua, 5) < 0)
        {
            perror("writeBytesSerialPort");
            closeSerialPort();
            return -1;
        }

        return 0;
    }

    // Transmitter: send SET and wait for UA with retries
    else if (connectionParameters.role == LlTx)
    {
        // install alarm handler
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = alarmHandler;
        sigemptyset(&sa.sa_mask);
        if (sigaction(SIGALRM, &sa, NULL) == -1)
        {
            perror("sigaction");
            closeSerialPort();
            return -1;
        }

    // Transmissor: construímos o quadro SET e tentamos enviar até
    // obter um UA em resposta. Se o timeout expirar, reenviamos
    // até `nRetransmissions` vezes.
    unsigned char set[5];
        set[0] = FLAG;
        set[1] = A_SET;
        set[2] = C_SET;
        set[3] = A_SET ^ C_SET;
        set[4] = FLAG;

        int attempts = 0;
        int maxAttempts = connectionParameters.nRetransmissions;
        int timeoutSec = connectionParameters.timeout;

        while (attempts <= maxAttempts)
        {
            if (writeBytesSerialPort(set, 5) < 0)
            {
                perror("writeBytesSerialPort");
                closeSerialPort();
                return -1;
            }

            alarmFired = 0;
            alarm(timeoutSec);

        State state = S_START;
            int got_ua = 0;

            while (!alarmFired && !got_ua)
            {
                int rv = readByteSerialPort(&byte);
                if (rv < 0)
                {
                    if (errno == EINTR)
                        break; // interrupted by alarm
                    perror("readByteSerialPort");
                    closeSerialPort();
                    return -1;
                }
                if (rv == 0)
                    continue;

                switch (state)
                {
                case S_START:
                    if (byte == FLAG)
                        state = S_FLAG;
                    break;
                case S_FLAG:
                    if (byte == A_UA)
                        state = S_A;
                    else if (byte == FLAG)
                        state = S_FLAG;
                    else
                        state = S_START;
                    break;
                case S_A:
                    if (byte == C_UA)
                        state = S_C;
                    else if (byte == FLAG)
                        state = S_FLAG;
                    else
                        state = S_START;
                    break;
                case S_C:
                    if (byte == (A_UA ^ C_UA))
                        state = S_BCC;
                    else if (byte == FLAG)
                        state = S_FLAG;
                    else
                        state = S_START;
                    break;
                case S_BCC:
                    if (byte == FLAG)
                        got_ua = 1;
                    else
                        state = S_START;
                    break;
                default:
                    state = S_START;
                    break;
                }
            }

            alarm(0);

            if (got_ua)
            {
                return 0; // success
            }

            attempts++;
        }

        // failed after retries
        closeSerialPort();
        return -1;
    }

    closeSerialPort();
    return -1;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    if (!link_opened)
        return -1; 

    // Calcular BCC2 primeiro
    unsigned char BCC2 = 0;
    if (bufSize > 0) {
        BCC2 = buf[0];
        for (int i = 1; i < bufSize; i++) BCC2 ^= buf[i];
    }

    // Alocar espaço suficiente para o frame (pior caso: tudo precisa de stuffing)
    int maxFrameSize = 6 + 2 * bufSize + 2; // margem extra para segurança
    unsigned char *frame = (unsigned char *) malloc(maxFrameSize);
    if (!frame) {
        perror("malloc");
        return -1;
    }
    
    // Construir cabeçalho
    frame[0] = FLAG;
    frame[1] = A_ER;
    frame[2] = C_N(seq_tx);
    frame[3] = frame[1] ^ frame[2];
    
    int j = 4;
    
    // Byte-stuffing dos dados
    for (int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG || buf[i] == ESC) {
            frame[j++] = ESC;
        }
        frame[j++] = buf[i];
    }
    
    // Byte-stuffing do BCC2 se necessário
    if (BCC2 == FLAG || BCC2 == ESC) {
        frame[j++] = ESC;
    }
    frame[j++] = BCC2;
    frame[j++] = FLAG;

    int currentTransmission = 0;
    int success = 0;

    while (currentTransmission < link_retrans && !success) {        
        // Enviar frame UMA VEZ
        if (writeBytesSerialPort(frame, j) < 0) {
            perror("writeBytesSerialPort");
            free(frame);
            return -1;
        }
        
        alarmFired = 0;
        alarm(link_timeout);
        
        // Ler resposta de controlo
        State state = S_START;
        unsigned char rbyte;
        int got_control = 0;
        unsigned char control_c = 0;
        unsigned char control_a = 0;

        while (!alarmFired && !got_control)
        {
            int rv = readByteSerialPort(&rbyte);
            if (rv < 0)
            {
                if (errno == EINTR) break;
                perror("readByteSerialPort");
                free(frame);
                return -1;
            }
            if (rv == 0) continue;

            switch (state)
            {
            case S_START:
                if (rbyte == FLAG) state = S_FLAG;
                break;
            case S_FLAG:
                control_a = rbyte; state = S_A; break;
            case S_A:
                control_c = rbyte; state = S_C; break;
            case S_C:
                if (rbyte == (control_a ^ control_c)) state = S_BCC;
                else state = S_START;
                break;
            case S_BCC:
                if (rbyte == FLAG) { got_control = 1; }
                else state = S_START;
                break;
            default:
                state = S_START; break;
            }
        }
        
        alarm(0);
        
        if (got_control)
        {
            // Verificar se é o RR correto para a nossa sequência
            unsigned char expected_rr = C_RR((seq_tx + 1) % 2);
            
            if (control_c == expected_rr) {
                success = 1;
                seq_tx = (seq_tx + 1) % 2;
            }
            else if (control_c == C_REJ(seq_tx)) {
                // REJ para a nossa sequência - retransmitir
                printf("REJ received for seq %d, retransmitting\n", seq_tx);
            }
            // Se receber RR da sequência anterior, ignorar (frame duplicado foi aceite)
        }
        
        currentTransmission++;
    }
    
    free(frame);
    
    if (success) return bufSize;
    else return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    if (!link_opened)
        return -1;

    unsigned char byte, cField;
    int i = 0;
    LinkLayerState state = START;

    // Ciclo principal: lê bytes até completar receção de um I-frame ou DISC
    while (state != STOP_R) {
        int rv = readByteSerialPort(&byte);
        if (rv < 0) {
            if (errno == EINTR) continue;
            perror("readByteSerialPort");
            return -1;
        }
        if (rv == 0) continue; // nenhum byte lido

        switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;
                
            case FLAG_RCV:
                if (byte == A_ER) state = A_RCV;
                else if (byte != FLAG) state = START;
                break;
                
            case A_RCV:
                // Verifica se é um I-frame com sequência 0 ou 1
                if (byte == C_N(0) || byte == C_N(1)) {
                    state = C_RCV;
                    cField = byte;
                }
                else if (byte == FLAG) state = FLAG_RCV;
                // Trata quadro DISC (desligamento)
                else if (byte == C_DISC) {
                    sendSupervisionFrame(A_RE, C_DISC);
                    return 0; // sinaliza desligamento
                }
                else state = START;
                break;
                
            case C_RCV:
                // Verifica BCC1 = A_ER ^ cField
                if (byte == (A_ER ^ cField)) state = READING_DATA;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;
                
            case READING_DATA:
                if (byte == ESC) {
                    state = DATA_FOUND_ESC;
                }
                else if (byte == FLAG) {
                    // Fim do payload: validar BCC2
                    if (i < 1) {
                        // Payload vazio - BCC2 deve ser 0
                        printf("Empty payload received\n");
                        i = 0;
                        state = START;
                        break;
                    }
                    
                    unsigned char bcc2 = packet[i-1];
                    i--; // remove BCC2 do payload
                    
                    // Calcular BCC2 esperado (XOR de todo o payload)
                    unsigned char acc = 0;
                    if (i > 0) {
                        acc = packet[0];
                        for (int j = 1; j < i; j++)
                            acc ^= packet[j];
                    }

                    // Verificar se o I-frame tem a sequência esperada
                    unsigned char expected_seq = (cField == C_N(0)) ? 0 : 1;

                    if (bcc2 == acc && expected_seq == seq_rx) {
                        // BCC2 correto e sequência esperada: enviar RR do próximo esperado
                        state = STOP_R;
                        seq_rx = (seq_rx + 1) % 2; // alternar sequência esperada
                        sendSupervisionFrame(A_RE, C_RR(seq_rx)); // RR do próximo esperado
                        return i; // retornar comprimento do payload
                    }
                    else if (bcc2 != acc) {
                        // Erro de BCC2: enviar REJ
                        sendSupervisionFrame(A_RE, C_REJ(seq_rx));
                        return -1;
                    }
                    else {
                        // Sequência duplicada: enviar RR do que já foi aceite
                        sendSupervisionFrame(A_RE, C_RR(seq_rx));
                        i = 0; // reset buffer
                        state = START; // voltar ao início
                        break;
                    }
                }
                else {
                    // Byte normal do payload
                    packet[i++] = byte;
                }
                break;
                
            case DATA_FOUND_ESC:
                state = READING_DATA;
                // Destuffing: se o próximo byte é FLAG ou ESC, é um byte "stuffed"
                if (byte == FLAG || byte == ESC) {
                    packet[i++] = byte;
                }
                else {
                    // Erro de stuffing: ESC seguido de byte inválido
                    packet[i++] = ESC;
                    packet[i++] = byte;
                }
                break;
                
            default:
                state = START;
                break;
        }
    }
    
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    if (!link_opened)
        return -1;

    LinkLayerState state = START;
    unsigned char byte;
    
    // Instalar handler do alarme (se ainda não estiver instalado)
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = alarmHandler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGALRM, &sa, NULL);
    
    int retransmitions = link_retrans;
    
    // Ciclo de retransmissão: enviar DISC e aguardar resposta DISC
    while (retransmitions > 0 && state != STOP_R) {
        
        // Enviar quadro DISC 
        if (sendSupervisionFrame(A_ER, C_DISC) < 0) {
            perror("sendSupervisionFrame DISC");
            return -1;
        }
        
        alarmFired = 0;
        alarm(link_timeout);
        
        // Aguardar resposta DISC do receptor
        while (alarmFired == 0 && state != STOP_R) {
            int rv = readByteSerialPort(&byte);
            if (rv < 0) {
                if (errno == EINTR) break; // alarme disparou
                perror("readByteSerialPort");
                return -1;
            }
            if (rv == 0) continue; // nenhum byte lido
            
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_RE) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == C_DISC) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (A_RE ^ C_DISC)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (byte == FLAG) state = STOP_R;
                    else state = START;
                    break;
                default: 
                    break;
            }
        }
        
        alarm(0); // cancelar alarme
        retransmitions--;
    }

    // Verificar se conseguimos receber o DISC de resposta
    if (state != STOP_R) {
        printf("Error: failed to receive DISC response\n");
        closeSerialPort();
        return -1;
    }
    
    // Enviar UA final para confirmar o desligamento
    if (sendSupervisionFrame(A_ER, C_UA) < 0) {
        perror("sendSupervisionFrame UA");
        closeSerialPort();
        return -1;
    }
    
    // Fechar porta série e marcar como fechada
    link_opened = 0;
    closeSerialPort();
    return 0;
}
