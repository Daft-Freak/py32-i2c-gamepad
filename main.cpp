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

static uint16_t adc_val[2];

static void init_adc()
{
    // enable ADC clock
    RCC->APBENR2 |= RCC_APBENR2_ADCEN;

    // setup ADC

    // EOCIE intr in ADC_IER?

    // calibrate
    ADC1->CR = 0;
    ADC1->CR = ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);

    delay_ms(1);

    ADC1->CFGR1 = ADC_CFGR1_DISCEN;
    ADC1->CFGR2 = 0; // div = 1?
    ADC1->SMPR = 6; // 71.5 cycles
    ADC1->CHSELR = 3 << 8; // B0/1

    ADC1->CR = ADC_CR_ADEN;

    // enable GPIO clock
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // setup IO
    int adc_x = 0; // B
    int adc_y = 1;
    const int mode_analog = 3;

    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE0 << (adc_x * 2) | GPIO_MODER_MODE0 << (adc_y * 2)))
                 | mode_analog << (adc_x * 2) | mode_analog << (adc_y * 2);
}

int main()
{
    init_hsi();
    init_systick();
    init_uart(115200);
    init_adc();

    uart_puts("testing!\n");

    while(true)
    {
        // read ADC
        uint16_t new_val[2];
        for(int i = 0; i < 2; i++)
        {
            ADC1->CR = ADC_CR_ADEN; // needing to re-enable seems strange?
            ADC1->CR = ADC_CR_ADSTART;

            while(!(ADC1->ISR & ADC_ISR_EOC));
            ADC1->ISR = ADC_ISR_EOC;

            new_val[i] = ADC1->DR;
        }

        if(new_val[0] != adc_val[0] || new_val[1] != adc_val[1])
        {
            adc_val[0] = new_val[0];
            adc_val[1] = new_val[1];
            uart_printf("ADC: %02X %02X\n", adc_val[0], adc_val[1]);
        }

        delay_ms(100);
    }

    return 0;
}