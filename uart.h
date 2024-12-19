#pragma once
#ifdef __cplusplus
extern "C" {
#endif

void init_uart(int baud);

void uart_putc(char c);
void uart_puts(const char *s);
int uart_printf(const char *format, ...);

#ifdef __cplusplus
}
#endif