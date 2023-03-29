#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <time.h>

#include "mh-z19b.h"


#define UART_ID uart0
#define BAUD_RATE 9600

#define UART_TX_PIN 0
#define UART_RX_PIN 1


int uart_tx(const uint8_t* buf, const int len){
    uart_write_blocking(UART_ID, buf, len);
    return len;
}

int uart_rx(uint8_t* buf, const int len, const int time_out_ms){
    int bytes_read = 0;
    int64_t time_left = time_out_ms * 1000;
    uint64_t ts_end = time_us_64() + time_out_ms * 1000;
    do{
        if(uart_is_readable_within_us(UART_ID, time_left)){
            buf[bytes_read] = uart_getc(UART_ID);
            ++bytes_read;
        }
        time_left = (ts_end - time_us_64());
    }while(bytes_read < len && time_left > 0);
    return bytes_read;
}

void uart_discard_input(void){
    while(uart_is_readable(UART_ID)){
        uart_getc(UART_ID);
    }
    return;
}

int main() {
    stdio_init_all();
	
	
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    mh_z19b_init(&uart_rx, &uart_tx, &uart_discard_input);
	
    while(1){
        int co2_ppm = mh_z19b_get_c02_ppm();
        if(co2_ppm < 0){
            printf("mh_z19b_get_c02_ppm failed\n");
        }else{
            printf("co2 ppm %d\n", co2_ppm);
        }
        sleep_ms(5000);
    }
}
