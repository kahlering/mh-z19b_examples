#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <time.h>

#include "mh-z19b.h"

#define UART_ID uart0
#define BAUD_RATE 9600

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1


int uart_tx(const uint8_t* buf, const int len){
    uart_write_blocking(UART_ID, buf, len);
    return len;
}

int uart_rx(uint8_t* buf, const int len, const int time_out_ms){
    int widx = 0;
    uint64_t time_left = time_out_ms;
    uint64_t ts_end = time_us_64() + time_out_ms * 1000;
    do{
        if(uart_is_readable_within_us(UART_ID, time_left * 1000)){
            buf[widx] = uart_getc(UART_ID);
        }
        time_left = (time_us_64() - ts_end) / 1000;
    }while(widx < len && time_left > 0);
    return widx;
}

void uart_discard_input(void){
    while(uart_is_readable(UART_ID)){
        uart_getc(UART_ID);
    }
    return;
}

int main() {
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    mh_z19b_init(&uart_rx, &uart_tx, &uart_discard_input);

    while(1){
        int co2_ppm = mh_z19b_get_c02_ppm();
        if(co2_ppm < 0){
            printf("mh_z19b_get_c02_ppm failed\n");
        }else{
            printf("co2 ppm %d", co2_ppm);
        }
        sleep_ms(5000);
    }
}

/// \end::hello_uart[]
