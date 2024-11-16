#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "py32f0xx.h"

// gpio helpers
enum gpio_mode
{
    GPIO_MODE_INPUT     = 0,
    GPIO_MODE_OUTPUT    = 1,
    GPIO_MODE_ALTERNATE = 2,
    GPIO_MODE_ANALOG    = 3,
};

enum gpio_speed
{
    GPIO_SPEED_VERY_LOW  = 0,
    GPIO_SPEED_LOW       = 1,
    GPIO_SPEED_HIGH      = 2,
    GPIO_SPEED_VERY_HIGH = 3,
};

enum gpio_pull
{
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP   = 1,
    GPIO_PULL_DOWN = 2,
};

inline void gpio_set_mode(GPIO_TypeDef *port, int pin, int mode)
{
    port->MODER = (port->MODER & ~(GPIO_MODER_MODE0 << (pin * 2)))
                | mode << (pin * 2);
}

inline void gpio_set_speed(GPIO_TypeDef *port, int pin, int speed)
{
    port->OSPEEDR = (port->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED0 << (pin * 2)))
                  | speed << (pin * 2);
}

inline void gpio_set_pulls(GPIO_TypeDef *port, int pin, int pulls)
{
    port->PUPDR = (port->PUPDR & ~(GPIO_PUPDR_PUPD0 << (pin * 2)))
                | pulls << (pin * 2);
}

inline void gpio_set_function(GPIO_TypeDef *port, int pin, int func)
{
    port->AFR[pin / 8] = (port->AFR[pin / 8] & ~(GPIO_AFRL_AFSEL0 << ((pin % 8) * 4)))
                       | func << ((pin % 8) * 4);
}

inline uint16_t gpio_get(GPIO_TypeDef *port)
{
    return port->IDR;
}

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

static uint8_t i2c_read_data[4];
static int i2c_read_offset = 0;

static void init_i2c_slave()
{
    // enable I2C clock
    RCC->APBENR1 |= RCC_APBENR1_I2CEN;

    I2C1->CR2 = I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | 8 << I2C_CR2_FREQ_Pos;
    I2C1->OAR1 = 0x55 << 1;
    I2C1->CR1 = I2C_CR1_PE;

    // enable GPIO clock
    RCC->IOPENR |= RCC_IOPENR_GPIOFEN;

    // setup IO
    const int sda = 0; //F
    const int scl = 1;
    const int alt_func_i2c = 12;
    gpio_set_mode(GPIOF, sda, GPIO_MODE_ALTERNATE);
    gpio_set_mode(GPIOF, scl, GPIO_MODE_ALTERNATE);
    gpio_set_speed(GPIOF, sda, GPIO_SPEED_VERY_LOW);
    gpio_set_speed(GPIOF, scl, GPIO_SPEED_VERY_LOW);
    gpio_set_function(GPIOF, sda, alt_func_i2c);
    gpio_set_function(GPIOF, scl, alt_func_i2c);

    // enable IRQ
    NVIC_SetPriority(I2C1_IRQn, 0);
    NVIC_EnableIRQ(I2C1_IRQn);

    I2C1->CR1 |= I2C_CR1_ACK;
}

extern "C"
void I2C1_IRQHandler()
{
    auto status1 = I2C1->SR1;
    auto status2 = I2C1->SR2;


    if(status1 & I2C_SR1_AF)
    {
        // nack
        I2C1->SR1 &= ~I2C_SR1_AF;
    }

    if(status1 & I2C_SR1_STOPF)
    {
        // stop
        I2C1->CR1 = I2C1->CR1; // need to write CR1 to clear STOP
    }

    if(status1 & I2C_SR1_BTF)
    {
        // byte trans done

        if(status2 & I2C_SR2_TRA)
        {
            I2C1->DR = i2c_read_data[i2c_read_offset]; // keep writing
            i2c_read_offset = (i2c_read_offset + 1) % 4;
        }
    }

    if(status1 & I2C_SR1_ADDR)
    {
        // got addr
        if(status2 & I2C_SR2_TRA)
        {
            i2c_read_offset = 1;
            I2C1->DR = i2c_read_data[0]; // this is a read, so we're writing
        }
    }
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

    gpio_set_mode(GPIOB, adc_x, GPIO_MODE_ANALOG);
    gpio_set_mode(GPIOB, adc_y, GPIO_MODE_ANALOG);
}

int main()
{
    init_hsi();
    init_systick();
    init_uart(115200);
    init_adc();
    init_i2c_slave();

    // more inputs
    gpio_set_mode(GPIOA, 0, GPIO_MODE_ALTERNATE);
    gpio_set_pulls(GPIOA, 0, GPIO_PULL_UP);
    gpio_set_mode(GPIOA, 1, GPIO_MODE_ALTERNATE);
    gpio_set_pulls(GPIOA, 1, GPIO_PULL_UP);

    for(int i = 4; i < 8; i++) {
        gpio_set_mode(GPIOA, i, GPIO_MODE_ALTERNATE);
        gpio_set_pulls(GPIOA, i, GPIO_PULL_UP);
    }

    uart_puts("testing!\n");

    uint16_t inputs = 0;

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

        if(std::abs(new_val[0] - adc_val[0]) > 8 || std::abs(new_val[1] - adc_val[1]) > 8)
        {
            adc_val[0] = new_val[0];
            adc_val[1] = new_val[1];
            //uart_printf("ADC: %02X %02X\n", adc_val[0], adc_val[1]);
        }

        auto new_inputs = gpio_get(GPIOA);

        auto changed = new_inputs ^ inputs;
        inputs = new_inputs;

        if(changed)
        {
            for(int i = 0; i < 8; i++)
            {
                if(changed & (1 << i))
                    uart_printf("i %i: %i\n", i, new_inputs & (1 << i));
            }
        }

        // merge adc+inputs into a single word
        *(uint32_t *)i2c_read_data = adc_val[0] | adc_val[1] << 16 | (inputs & 0xF) << 12 | (inputs & 0xF0) << 24;

        delay_ms(100);
    }

    return 0;
}