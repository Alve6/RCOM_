// Example of how to read from the serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BAUDRATE 38400
#define BUF_SIZE 256

// Trama: FLAG A C BCC FLAG
#define FLAG 0x7E
#define A    0x03
#define C_SET 0x03
#define C_UA  0x07

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State;

int fd = -1;           // File descriptor for open serial port
struct termios oldtio; // Serial port settings to restore on closing

int openSerialPort(const char *serialPort, int baudRate);
int closeSerialPort();
int readByteSerialPort(unsigned char *byte);
int writeBytesSerialPort(const unsigned char *bytes, int nBytes);

// ---------------------------------------------------
// MAIN
// ---------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS0\n",
               argv[0], argv[0]);
        exit(1);
    }

    const char *serialPort = argv[1];

    if (openSerialPort(serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }

    printf("Serial port %s opened\n", serialPort);

    // Máquina de estados para receber SET
    State state = START;
    unsigned char byte;

    while (state != STOP) {
        int res = readByteSerialPort(&byte);
        if (res <= 0) continue; // sem bytes, tenta outra vez

        switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == A) state = A_RCV;
                else if (byte != FLAG) state = START;
                break;

            case A_RCV:
                if (byte == C_SET) state = C_RCV;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;

            case C_RCV:
                if (byte == (A ^ C_SET)) state = BCC_OK;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;

            case BCC_OK:
                if (byte == FLAG) state = STOP;
                else state = START;
                break;

            default:
                state = START;
                break;
        }
    }

    printf("SET recebido corretamente!\n");

    unsigned char ua[5];
    ua[0] = FLAG;
    ua[1] = 0x01;             
    ua[2] = 0x07;             
    ua[3] = ua[1] ^ ua[2];    
    ua[4] = FLAG;

    int bytes = writeBytesSerialPort(ua, 5);
    printf("UA enviado (%d bytes)\n", bytes);

    // Fecha a porta série
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    printf("Serial port %s closed\n", serialPort);

    return 0;
}

// ---------------------------------------------------
// SERIAL PORT LIBRARY IMPLEMENTATION
// ---------------------------------------------------

int openSerialPort(const char *serialPort, int baudRate)
{
    int oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    fd = open(serialPort, oflags);
    if (fd < 0) {
        perror(serialPort);
        return -1;
    }

    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

#define CASE_BAUDRATE(baudrate) \
    case baudrate:              \
        br = B##baudrate;       \
        break;

    tcflag_t br;
    switch (baudRate) {
        CASE_BAUDRATE(1200);
        CASE_BAUDRATE(1800);
        CASE_BAUDRATE(2400);
        CASE_BAUDRATE(4800);
        CASE_BAUDRATE(9600);
        CASE_BAUDRATE(19200);
        CASE_BAUDRATE(38400);
        CASE_BAUDRATE(57600);
        CASE_BAUDRATE(115200);
    default:
        fprintf(stderr, "Unsupported baud rate\n");
        return -1;
    }
#undef CASE_BAUDRATE

    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = br | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    oflags ^= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, oflags) == -1) {
        perror("fcntl");
        close(fd);
        return -1;
    }

    return fd;
}

int closeSerialPort()
{
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    return close(fd);
}

int readByteSerialPort(unsigned char *byte)
{
    return read(fd, byte, 1);
}

int writeBytesSerialPort(const unsigned char *bytes, int nBytes)
{
    return write(fd, bytes, nBytes);
}
