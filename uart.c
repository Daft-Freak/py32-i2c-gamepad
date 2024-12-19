#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "py32f0xx.h"

#include "gpio.h"

void init_uart(int baud)
{
    // enable USART clock
    RCC->APBENR2 |= RCC_APBENR2_USART1EN;

    // setup USART
    USART1->BRR = 8000000 / baud; // 4 bit fraction, but also / 16
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    USART1->CR1 = USART_CR1_UE;

    // enable GPIO clock
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    // setup IO
    int rx = 3;
    int tx = 2;
    const int alt_func_uart = 1;

    gpio_set_mode(GPIOA, rx, GPIO_MODE_ALTERNATE);
    gpio_set_mode(GPIOA, tx, GPIO_MODE_ALTERNATE);
    gpio_set_speed(GPIOA, rx, GPIO_SPEED_VERY_LOW);
    gpio_set_speed(GPIOA, tx, GPIO_SPEED_VERY_LOW);
    gpio_set_pulls(GPIOA, rx, GPIO_PULL_UP);
    gpio_set_pulls(GPIOA, tx, GPIO_PULL_UP);
    gpio_set_function(GPIOA, rx, alt_func_uart);
    gpio_set_function(GPIOA, tx, alt_func_uart);
}

void uart_putc(char c)
{
    USART1->CR1 |= USART_CR1_TE;
    USART1->SR &= ~USART_SR_TC;

    USART1->DR = c;

    while(!(USART1->SR & USART_SR_TC));

    USART1->CR1 &= ~USART_CR1_TE;
}

void uart_puts(const char *s)
{
    while(*s)
        uart_putc(*s++);
}

int uart_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // get length
    va_list tmp_args;
    va_copy(tmp_args, args);
    int len = vsnprintf(NULL, 0, format, tmp_args) + 1;
    va_end(tmp_args);

    char * buf = malloc(len);
    int ret = vsnprintf(buf, len, format, args);
    uart_puts(buf);
    va_end(args);

    free(buf);
    return ret;
}
