#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sys/epoll.h>
#include <cstring>
#include <chrono>

#include "mh-z19b.h"



#define MAX_EVENTS 10

// https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts

// first PL011
//#define UART_DEVICE "/dev/ttyAMA0"

// mini UART
#define UART_DEVICE "/dev/ttyS0"

static struct epoll_event event, events[MAX_EVENTS];
static int epoll_fd;
static int uart_fd;


static int open_uart(const char* device){
    int device_fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);

    if (device_fd == -1) {
        return -1;
    }

    struct termios tty;

    if (tcgetattr(device_fd, &tty) < 0) {
        return -1;
    }

    cfsetospeed(&tty, (speed_t)B9600);
    cfsetispeed(&tty, (speed_t)B9600);

    tty.c_cflag |= (CLOCAL | CREAD);  // no modem controls
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8-bit characters
    tty.c_cflag &= ~PARENB;   // no parity bit
    tty.c_cflag &= ~CSTOPB;   // only need 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // no hardware flowcontrol

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;   // wait for at least n bytes to arrive
    tty.c_cc[VTIME] = 0;  // 0.1s timeout. timer resets if a new byte is received.

    if (tcsetattr(device_fd, TCSANOW, &tty) != 0) {
        return -1;
    }

    return device_fd;
}


static int uart_tx(const uint8_t* buf, const int len){
    return write(uart_fd, buf, len);
}

static int uart_rx(uint8_t* buf, const int len, const int time_out_ms){
    auto ts_end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(time_out_ms);
    int time_left = time_out_ms;
    int bytes_read = 0;
    do{
        int event_count = epoll_wait(epoll_fd, events, MAX_EVENTS, time_left);
        if(event_count < 0){
            return -1;
        }
        for(int i = 0; i < event_count && bytes_read <= len; ++i){
            int t = read(events[i].data.fd, buf + bytes_read, len-bytes_read);
            if(t < 0){
                return -1;
            }
            bytes_read += t;
        }
        auto ts_now = std::chrono::high_resolution_clock::now();
        time_left = std::chrono::duration_cast<std::chrono::milliseconds>(ts_end - ts_now).count();
    }while(bytes_read < len &&  time_left > 0);
    return bytes_read;
}


static void uart_discard_input(){
    tcflush(uart_fd, TCIFLUSH);
}

static void epoll_init(int fd){
    epoll_fd = epoll_create1(0);
    if(epoll_fd < 0){
        exit(EXIT_FAILURE);
    }

    memset(&event, 0, sizeof(event));
    event.events = EPOLLIN;
    event.data.fd = fd;

    int r = epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &event);
    if(r < 0){
        close(epoll_fd);
        exit(EXIT_FAILURE);
    }
}


int main(){
    uart_fd = open_uart(UART_DEVICE);
    if(uart_fd < 0){
        std::cout << "failed to open uart device" << std::endl;
        exit(EXIT_FAILURE);
    }

    epoll_init(uart_fd);

    mh_z19b_init(&uart_rx, &uart_tx, &uart_discard_input);

    while(true){
        int co2_ppm = mh_z19b_get_c02_ppm();
        if(co2_ppm < 0){
            std::cout << "mh_z19b_get_c02_ppm failed" << std::endl;
        }else{
            std::cout << "co2 ppm " << co2_ppm << std::endl;
        }
        sleep(5);
    }
    
}