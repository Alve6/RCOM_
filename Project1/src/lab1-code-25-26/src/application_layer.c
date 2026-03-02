// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

// Function prototypes
unsigned char* parseControlPacket(unsigned char* packet, int size, unsigned long int *fileSize);
unsigned char* getControlPacket(const unsigned int c, const char* filename, long int length, unsigned int* size);
unsigned char* getDataPacket(unsigned char sequence, unsigned char *data, int dataSize, int *packetSize);
unsigned char* getData(FILE* fd, long int fileLength);
void parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer);

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Configurar estrutura da camada de enlace
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
    // strcmp retorna 0 se strings são iguais, então: se role="tx" → LlTx, senão LlRx
    linkLayer.role = strcmp(role, "tx") ? LlRx : LlTx;
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    printf("Starting file transfer protocol application\n");
    printf("  - Serial port: %s\n", serialPort);
    printf("  - Role: %s\n", role);
    printf("  - File: %s\n", filename);
    
    int res = llopen(linkLayer);
    if (res < 0) {
        perror("Connection error\n");
        exit(-1);
    }

    switch (linkLayer.role) {

        case LlTx: {
            
            FILE* file = fopen(filename, "rb");
            if (file == NULL) {
                perror("File not found\n");
                exit(-1);
            }

            // Calcular tamanho do ficheiro:
            int prev = ftell(file);        // Guardar posição atual
            fseek(file, 0L, SEEK_END);     // Ir para o fim do ficheiro
            long int fileSize = ftell(file) - prev;  // Tamanho = posição_fim - posição_inicial
            fseek(file, prev, SEEK_SET);   // Voltar à posição original

            printf("Sending file '%s' (%ld bytes)...\n", filename, fileSize);

            // 1. Enviar pacote de controlo START (tipo 2) com nome e tamanho do ficheiro
            unsigned int cpSize;
            unsigned char *controlPacketStart = getControlPacket(2, filename, fileSize, &cpSize);
            if(llwrite(controlPacketStart, cpSize) == -1){ 
                printf("Exit: error in start packet\n");
                exit(-1);
            }

            // 2. Enviar dados do ficheiro em pacotes DATA
            unsigned char sequence = 0;             // Número de sequência dos pacotes (0-254)
            unsigned char* content = getData(file, fileSize);  // Ler todo o ficheiro para memória
            long int bytesLeft = fileSize;          // Bytes que ainda faltam enviar

            while (bytesLeft > 0) { 
                // Determinar tamanho do próximo pedaço: MAX_PAYLOAD_SIZE ou o que resta
                int dataSize = bytesLeft > (long int) MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : bytesLeft;
                
                // Copiar pedaço do ficheiro para buffer temporário
                unsigned char* data = (unsigned char*) malloc(dataSize);
                memcpy(data, content, dataSize);
                
                // Criar pacote DATA com cabeçalho (tipo=1, sequência, tamanho)
                int packetSize;
                unsigned char* packet = getDataPacket(sequence, data, dataSize, &packetSize);
                
                // Enviar pacote pela camada de enlace
                if(llwrite(packet, packetSize) == -1) {
                    printf("Exit: error in data packets\n");
                    exit(-1);
                }
                
                // Avançar para próximo pedaço
                bytesLeft -= (long int) dataSize; 
                content += dataSize;               // Avançar ponteiro no buffer
                sequence = (sequence + 1) % 255;   // Incrementar sequência (máx 254)
                
                free(data);
                free(packet);
            }

            // 3. Enviar pacote de controlo END (tipo 3) para sinalizar fim da transmissão
            unsigned char *controlPacketEnd = getControlPacket(3, filename, fileSize, &cpSize);
            if(llwrite(controlPacketEnd, cpSize) == -1) { 
                printf("Exit: error in end packet\n");
                exit(-1);
            }
            
            printf("File sent successfully!\n");
            llclose();
            fclose(file);
            break;
        }

        case LlRx: {

            // Buffer para receber pacotes da camada de enlace
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            int packetSize = -1;
            
            // 1. Aguardar pacote START do transmissor
            while ((packetSize = llread(packet)) <= 0) {
                if (packetSize == 0) {  // 0 = DISC recebido, ligação fechada
                    printf("Connection terminated by peer\n");
                    return;
                }
                // packetSize < 0 = erro, tenta novamente
            }
            
            // Extrair nome e tamanho do ficheiro do pacote START
            unsigned long int rxFileSize = 0;
            unsigned char* name = parseControlPacket(packet, packetSize, &rxFileSize); 

            printf("Receiving file '%s' (%lu bytes)...\n", name, rxFileSize);

            FILE* newFile = fopen((char *) name, "wb+");
            if (newFile == NULL) {
                perror("Error creating output file");
                exit(-1);
            }
            
            // 2. Loop principal: receber pacotes DATA até receber pacote END
            while (1) {    
                packetSize = llread(packet);
                if(packetSize == 0) break;          // DISC recebido - ligação fechada
                else if(packetSize < 0) continue;   // Erro na receção - tentar novamente
                else if(packet[0] == 3) break;      // Pacote END (tipo 3) - fim da transmissão
                else if(packet[0] == 1) {           // Pacote DATA (tipo 1)
                    // Extrair dados do pacote (sem os 4 bytes de cabeçalho)
                    unsigned char *buffer = (unsigned char*)malloc(packetSize - 4);
                    parseDataPacket(packet, packetSize, buffer);  // Copia dados para buffer
                    fwrite(buffer, sizeof(unsigned char), packetSize - 4, newFile);  // Escrever no ficheiro
                    free(buffer);
                }
                // Outros tipos de pacotes são ignorados
            }

            printf("File received successfully!\n");
            fclose(newFile);
            llclose();  
            free(packet);
            free(name);
            break;
        }

        default:
            exit(-1);
            break;
    }
}

// Função para extrair informações de um pacote de controlo (START ou END)
// Formato: [tipo][T1][L1][tamanho_ficheiro][T2][L2][nome_ficheiro]
unsigned char* parseControlPacket(unsigned char* packet, int size, unsigned long int *fileSize) {

    // Extrair tamanho do ficheiro
    unsigned char fileSizeNBytes = packet[2];           // L1 = número de bytes do tamanho
    unsigned char fileSizeAux[fileSizeNBytes];
    memcpy(fileSizeAux, packet+3, fileSizeNBytes);      // Copiar bytes do tamanho
    *fileSize = 0;
    // Reconstruir tamanho: bytes estão em big-endian, converter para little-endian
    for(unsigned int i = 0; i < fileSizeNBytes; i++)
        *fileSize |= (fileSizeAux[fileSizeNBytes-i-1] << (8*i));

    // Extrair nome do ficheiro
    unsigned char fileNameNBytes = packet[3+fileSizeNBytes+1];  // L2 = comprimento do nome
    unsigned char *name = (unsigned char*)malloc(fileNameNBytes + 1);
    memcpy(name, packet+3+fileSizeNBytes+2, fileNameNBytes);    // Copiar nome
    name[fileNameNBytes] = '\0'; // Terminar string com null
    return name;
}

// Função para criar pacote de controlo (START=2 ou END=3)
// Formato: [tipo][T1=0][L1][tamanho_ficheiro][T2=1][L2][nome_ficheiro]
unsigned char * getControlPacket(const unsigned int c, const char* filename, long int length, unsigned int* size){

    // Calcular quantos bytes precisamos para representar o tamanho do ficheiro
    const int L1 = (int) ceil(log2f((float)length)/8.0);  // Bytes necessários para o tamanho
    const int L2 = strlen(filename);                       // Bytes do nome do ficheiro
    *size = 1+2+L1+2+L2;  // Tamanho total: tipo + T1 + L1 + tamanho + T2 + L2 + nome
    unsigned char *packet = (unsigned char*)malloc(*size);
    
    unsigned int pos = 0;
    packet[pos++] = c;    // Tipo do pacote (2=START, 3=END)
    packet[pos++] = 0;    // T1 = 0 (tipo do campo tamanho)
    packet[pos++] = L1;   // L1 = número de bytes do tamanho

    // Escrever tamanho do ficheiro em big-endian
    for (unsigned char i = 0 ; i < L1 ; i++) {
        packet[2+L1-i] = length & 0xFF;  // Byte menos significativo primeiro
        length >>= 8;                    // Shift para próximo byte
    }
    pos += L1;
    packet[pos++] = 1;    // T2 = 1 (tipo do campo nome)
    packet[pos++] = L2;   // L2 = comprimento do nome
    memcpy(packet+pos, filename, L2);   // Copiar nome do ficheiro
    return packet;
}

// Função para criar pacote de dados
// Formato: [tipo=1][sequência][tamanho_high][tamanho_low][dados...]
unsigned char * getDataPacket(unsigned char sequence, unsigned char *data, int dataSize, int *packetSize){

    *packetSize = 1 + 1 + 2 + dataSize;  // 4 bytes cabeçalho + dados
    unsigned char* packet = (unsigned char*)malloc(*packetSize);

    packet[0] = 1;                        // Tipo = 1 (pacote DATA)
    packet[1] = sequence;                 // Número de sequência (0-254)
    packet[2] = dataSize >> 8 & 0xFF;     // Byte mais significativo do tamanho
    packet[3] = dataSize & 0xFF;          // Byte menos significativo do tamanho
    memcpy(packet+4, data, dataSize);     // Copiar dados após cabeçalho

    return packet;
}

// Função para ler todo o conteúdo de um ficheiro para memória
unsigned char * getData(FILE* fd, long int fileLength) {
    unsigned char* content = (unsigned char*)malloc(sizeof(unsigned char) * fileLength);
    fread(content, sizeof(unsigned char), fileLength, fd);  // Ler fileLength bytes
    return content;
}

// Função para extrair dados de um pacote DATA (ignora os 4 bytes de cabeçalho)
void parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer) {
    memcpy(buffer, packet+4, packetSize-4);  // Copiar tudo exceto cabeçalho [tipo][seq][size_h][size_l]
}
