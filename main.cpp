#include <cstdarg>
#include <cstdio>

#include "py32f0xx.h"

static void init_hsi()
{
    // enable and wait
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // reset prescalers
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);

    // set sysclk src to HSI (should be default?)
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW); // HSISYS == 0
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    // HSI defaults to 8MHz, can configure to 24Mhz (RCC_ICSCR_HSI_FS)
}

static void init_uart(int baud)
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
    const int mode_alt_func = 2;
    const int alt_func_uart = 1;

    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE0 << (rx * 2) | GPIO_MODER_MODE0 << (tx * 2)))
                 | mode_alt_func << (rx * 2) | mode_alt_func << (tx * 2);
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (rx * 2) | GPIO_OSPEEDR_OSPEED0 << (tx * 2)); // lowest speed
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD0 << (rx * 2) | GPIO_PUPDR_PUPD0 << (tx * 2)))
                 | 1 << (rx * 2) | 1 << (tx * 2); // pull up
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL0 << (rx * 4) | GPIO_AFRL_AFSEL0 << (tx * 4)))
                  | alt_func_uart << (rx * 4) | alt_func_uart << (tx * 4);
}

static void init_systick()
{
    // 1ms at 8MHz
    SysTick->LOAD = 8000000 / 1000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static void delay_ms(int ms)
{
    (void)SysTick->CTRL; // clear flag

    // count flags
    while(ms)
    {
        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            ms--;
    }
}

static void uart_putc(char c)
{
    USART1->CR1 |= USART_CR1_TE;
    USART1->SR &= ~USART_SR_TC;

    USART1->DR = c;

    while(!(USART1->SR & USART_SR_TC));

    USART1->CR1 &= ~USART_CR1_TE;
}

static void uart_puts(const char *s)
{
    while(*s)
        uart_putc(*s++);
}

static int uart_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);

    // get length
    va_list tmp_args;
    va_copy(tmp_args, args);
    int len = vsnprintf(nullptr, 0, format, tmp_args) + 1;
    va_end(tmp_args);

    auto buf = new char[len];
    int ret = vsnprintf(buf, len, format, args);
    uart_puts(buf);
    va_end(args);

    delete[] buf;
    return ret;
}

int main()
{
    init_hsi();
    init_systick();
    init_uart(115200);

    uart_puts("testing!\n");

    while(true);

    return 0;
}