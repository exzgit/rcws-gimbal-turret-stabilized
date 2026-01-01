/*
 * host_control.c
 * Simple keyboard -> serial bridge in C (POSIX).
 * Build: gcc -o host_control host_control.c
 * Run:   ./host_control /dev/ttyUSB0
 *
 * Controls:
 *  - Left arrow  : send 'L'
 *  - Right arrow : send 'R'
 *  - Up arrow    : send 'U' (increase pitch)
 *  - Down arrow  : send 'D' (decrease pitch)
 *  - Space or s  : send 'S' (stop/neutral)
 *  - q or Ctrl-C : quit
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN]  = 0; // non-blocking read
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s /dev/ttyUSB0 [baud]\n", argv[0]);
        return 1;
    }

    const char *port = argv[1];
    int baud = (argc >= 3) ? atoi(argv[2]) : 115200;
    speed_t speed = B115200;
    if (baud == 9600) speed = B9600;
    else if (baud == 19200) speed = B19200;
    else if (baud == 38400) speed = B38400;
    else if (baud == 57600) speed = B57600;

    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", port, strerror(errno));
        return 1;
    }

    if (set_interface_attribs(fd, speed) != 0) {
        fprintf(stderr, "Failed to set serial attributes\n");
        close(fd);
        return 1;
    }

    struct termios orig_term;
    tcgetattr(STDIN_FILENO, &orig_term);
    struct termios raw = orig_term;
    cfmakeraw(&raw);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    printf("Connected to %s at %d baud\n", port, baud);
    printf("Use arrow keys (L/R/U/D), space or s to stop, q to quit\n");

    int running = 1;
    while (running) {
        char ch;
        ssize_t r = read(STDIN_FILENO, &ch, 1);
        if (r <= 0) {
            continue;
        }

        if (ch == '\x1b') {
            // escape sequence, read two more
            char seq[2];
            ssize_t r1 = read(STDIN_FILENO, &seq[0], 1);
            ssize_t r2 = read(STDIN_FILENO, &seq[1], 1);
            if (r1 <= 0 || r2 <= 0) continue;
            if (seq[0] == '[') {
                    if (seq[1] == 'D') {
                        write(fd, "L", 1);
                        printf("\r<- LEFT  "); fflush(stdout);
                    } else if (seq[1] == 'C') {
                        write(fd, "R", 1);
                        printf("\rRIGHT -> "); fflush(stdout);
                    } else if (seq[1] == 'A') {
                        write(fd, "U", 1);
                        printf("\rUP      "); fflush(stdout);
                    } else if (seq[1] == 'B') {
                        write(fd, "D", 1);
                        printf("\rDOWN    "); fflush(stdout);
                    }
            }
        } else {
            if (ch == 'q' || ch == 'Q') {
                running = 0;
                break;
            }
            if (ch == ' ' || ch == 's' || ch == 'S') {
                write(fd, "S", 1);
                printf("\rSTOP     "); fflush(stdout);
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);
    close(fd);
    printf("\nExiting\n");
    return 0;
}
