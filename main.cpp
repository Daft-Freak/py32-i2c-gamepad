#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "py32f0xx.h"

#include "gpio.h"
#include "uart.h"

// LED R/G pins wrong
// #define BROKEN_V1

#ifdef BROKEN_V1
#define I2C_ADDR_PIN 3
#else
#define I2C_ADDR_PIN 2
#endif

#define FLASH_CONFIG_ADDR (FLASH_END + 1 - FLASH_SECTOR_SIZE)
#define FLASH_CONFIG_MAGIC 0x55AA

// pow(x, 2.2)
static const uint16_t led_gamma_10[]
{
      0,    0,    0,    0,    0,    0,    0,    0,    1,    1,    1,    1,    1,    1,    2,    2,
      2,    3,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    9,    9,   10,
     11,   11,   12,   13,   14,   15,   16,   16,   17,   18,   19,   20,   21,   23,   24,   25,
     26,   27,   28,   30,   31,   32,   34,   35,   36,   38,   39,   41,   42,   44,   46,   47,
     49,   51,   52,   54,   56,   58,   60,   61,   63,   65,   67,   69,   71,   73,   76,   78,
     80,   82,   84,   87,   89,   91,   94,   96,   98,  101,  103,  106,  109,  111,  114,  117,
    119,  122,  125,  128,  130,  133,  136,  139,  142,  145,  148,  151,  155,  158,  161,  164,
    167,  171,  174,  177,  181,  184,  188,  191,  195,  198,  202,  206,  209,  213,  217,  221,
    225,  228,  232,  236,  240,  244,  248,  252,  257,  261,  265,  269,  274,  278,  282,  287,
    291,  295,  300,  304,  309,  314,  318,  323,  328,  333,  337,  342,  347,  352,  357,  362,
    367,  372,  377,  382,  387,  393,  398,  403,  408,  414,  419,  425,  430,  436,  441,  447,
    452,  458,  464,  470,  475,  481,  487,  493,  499,  505,  511,  517,  523,  529,  535,  542,
    548,  554,  561,  567,  573,  580,  586,  593,  599,  606,  613,  619,  626,  633,  640,  647,
    653,  660,  667,  674,  681,  689,  696,  703,  710,  717,  725,  732,  739,  747,  754,  762,
    769,  777,  784,  792,  800,  807,  815,  823,  831,  839,  847,  855,  863,  871,  879,  887,
    895,  903,  912,  920,  928,  937,  945,  954,  962,  971,  979,  988,  997, 1005, 1014, 1023
};

static uint8_t i2c_read_data[4];
static uint8_t i2c_write_data[16];
static int i2c_read_offset = 0, i2c_write_offset = 0;

static uint16_t adc_val[2];

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

extern "C" void SysTick_Handler()
{
}

static void init_systick()
{
    // 1ms at 8MHz
    SysTick->LOAD = 8000000 / 1000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

static void delay_ms(int ms)
{
    (void)SysTick->CTRL; // clear flag

    // count flags
    while(ms)
    {
        asm volatile("wfe");
        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            ms--;
    }
}

static void init_i2c_slave(uint8_t addr)
{
    // enable I2C clock
    RCC->APBENR1 |= RCC_APBENR1_I2CEN;

    I2C1->CR2 = I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | 8 << I2C_CR2_FREQ_Pos;
    I2C1->OAR1 = addr << 1;
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

    if(status1 & I2C_SR1_BERR)
    {
        // bus error
        I2C1->SR1 &= ~I2C_SR1_BERR;
    }

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

    // need to check this before rx/tx
    if(status1 & I2C_SR1_ADDR)
    {
        // got addr
        if(status2 & I2C_SR2_TRA)
            i2c_read_offset = 0;
        else
            i2c_write_offset = 0;
    }

    if(status1 & I2C_SR1_TXE)
    {
        // read the calibration data from the write buffer after the actual data
        auto data = i2c_read_offset >= 4 ? i2c_write_data : i2c_read_data;
        I2C1->DR = data[i2c_read_offset]; // keep writing
        i2c_read_offset = (i2c_read_offset + 1) % 16;
    }

    if(status1 & I2C_SR1_RXNE)
    {
        // RX not empty
        i2c_write_data[i2c_write_offset] = I2C1->DR;
        i2c_write_offset = (i2c_write_offset + 1) % 16;
    }
}

extern "C" void ADC_COMP_IRQHandler()
{
}

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
    ADC1->IER = ADC_IER_EOCIE;

    ADC1->CR = ADC_CR_ADEN;

    // enable GPIO clock
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // setup IO
    int adc_x = 0; // B
    int adc_y = 1;

    gpio_set_mode(GPIOB, adc_x, GPIO_MODE_ANALOG);
    gpio_set_mode(GPIOB, adc_y, GPIO_MODE_ANALOG);
}

static void init_pwm()
{
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;

    TIM1->CR1 |= TIM_CR1_CEN; // enable counter
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;

    TIM1->PSC = 8 - 1; // / 8 = 1MHz
    TIM1->ARR = 1024 - 1;
    TIM1->RCR = 0;

    TIM1->EGR = TIM_EGR_UG; // generate update

    TIM1->BDTR = TIM_BDTR_MOE; // enable outputs
    TIM1->CR1 |= TIM_CR1_CEN; // enable counter
    
    // setup IO
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    // PA3/AF13
    gpio_set_mode(GPIOA, 3, GPIO_MODE_ALTERNATE);
    gpio_set_output_type(GPIOA, 3, GPIO_OUTPUT_OPEN_DRAIN);
    gpio_set_function(GPIOA, 3, 13);

#ifndef BROKEN_V1
    // PB3/AF1
    gpio_set_mode(GPIOB, 3, GPIO_MODE_ALTERNATE);
    gpio_set_output_type(GPIOB, 3, GPIO_OUTPUT_OPEN_DRAIN);
    gpio_set_function(GPIOB, 3, 1);

    // PA0/AF13
    gpio_set_mode(GPIOA, 0, GPIO_MODE_ALTERNATE);
    gpio_set_output_type(GPIOA, 0, GPIO_OUTPUT_OPEN_DRAIN);
    gpio_set_function(GPIOA, 0, 13);
#else
    // non-PWM workaround pins
    gpio_set_mode(GPIOB, 2, GPIO_MODE_OUTPUT);
    gpio_set_output_type(GPIOB, 2, GPIO_OUTPUT_OPEN_DRAIN);

    gpio_set_mode(GPIOA, 5, GPIO_MODE_OUTPUT);
    gpio_set_output_type(GPIOA, 5, GPIO_OUTPUT_OPEN_DRAIN);
#endif

    // setup channels

    // disable channels
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);

    // setup ch1/2
    TIM1->CCMR1 = 7/*PWM 2*/ << TIM_CCMR1_OC1M_Pos | 7/*PWM 2*/ << TIM_CCMR1_OC2M_Pos;
    TIM1->CCR1 = 0; // compare value
    TIM1->CCR2 = 0; // compare value

    // setup ch3
    TIM1->CCMR2 = 7/*PWM 2*/ << TIM_CCMR2_OC3M_Pos;
    TIM1->CCR3 = 0;

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E; // enable
}

[[gnu::section(".ram_func")]] [[gnu::noinline]]
void write_flash_config(const uint16_t *data, unsigned len) {
    // disable irqs so we don't use flash
    __disable_irq();

    // this assumes that we're writing less than one page
    auto dest_ptr = (uint32_t *)FLASH_CONFIG_ADDR;
    auto in_32 = (uint32_t *)data;

    // unlock
    if(FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        // failed to unlock, give up
        if(FLASH->CR & FLASH_CR_LOCK)
            return;
    }

    // erase
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR |= FLASH_CR_PER;
    *dest_ptr = 0xFF;
    while(FLASH->SR & FLASH_SR_BSY);

    // write
    FLASH->CR |= FLASH_CR_PG;
    for(unsigned i = 0; i < FLASH_PAGE_SIZE; i += 4) {
        // start if last word
        if(i + 4 == FLASH_PAGE_SIZE)
            FLASH->CR |= FLASH_CR_PGSTRT;

        if(i < len)
            *dest_ptr++ = *in_32++;
        else
            *dest_ptr++ = 0xFFFFFFFF;
    }
    while(FLASH->SR & FLASH_SR_BSY);

    // lock
    FLASH->CR |= FLASH_CR_LOCK;

    __enable_irq();
}

int main()
{
    init_hsi();
    init_systick();

    // enable IO
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOFEN;

    // for I2C addr selection
    gpio_set_mode(GPIOB, I2C_ADDR_PIN, GPIO_MODE_INPUT);
    gpio_set_pulls(GPIOB, I2C_ADDR_PIN, GPIO_PULL_UP);

    uint8_t i2c_addr = 0x55;

    delay_ms(1);

    if(gpio_get(GPIOB) & (1 << I2C_ADDR_PIN))
        i2c_addr |= 2;

    //init_uart(115200);
    init_adc();
    init_pwm();
    init_i2c_slave(i2c_addr);

    // more inputs
    for(int i = 0; i < 8; i++)
    {
        if(i == 3) continue; // LED B (G on V1)
#ifdef BROKEN_V1
        if(i == 5) continue; // LED R
#else
        if(i == 0) continue; // LED G
#endif
        gpio_set_mode(GPIOA, i, GPIO_MODE_INPUT);
        gpio_set_pulls(GPIOA, i, GPIO_PULL_UP);
    }

#ifndef BROKEN_V1
    // used to be A0
    gpio_set_mode(GPIOB, 6, GPIO_MODE_INPUT);
    gpio_set_mode(GPIOF, 4, GPIO_MODE_INPUT); // pin is mapped to both
    gpio_set_pulls(GPIOB, 6, GPIO_PULL_UP);
    gpio_set_pulls(GPIOF, 4, GPIO_PULL_UP);
#endif

    // init calibration
    auto calib_min = (uint16_t *)(i2c_write_data + 4);
    auto calib_max = (uint16_t *)(i2c_write_data + 8);

    auto flash_calib = (uint16_t *)FLASH_CONFIG_ADDR;

    if(flash_calib[0] == FLASH_CONFIG_MAGIC) {
        calib_min[0] = flash_calib[1];
        calib_min[1] = flash_calib[2];
        calib_max[0] = flash_calib[3];
        calib_max[1] = flash_calib[4];
    } else {
        calib_min[0] = calib_min[1] = 0;
        calib_max[0] = calib_max[1] = 1 << 12;
    }

    uint16_t inputs = 0;

    while(true)
    {
        if(i2c_write_data[3] == 0xA5) {
            i2c_write_data[3] = 0;

            // save calibration
            uint16_t save_buf[5];
            save_buf[0] = FLASH_CONFIG_MAGIC;
            save_buf[1] = calib_min[0];
            save_buf[2] = calib_min[1];
            save_buf[3] = calib_max[0];
            save_buf[4] = calib_max[1];
            write_flash_config(save_buf, 5 * sizeof(uint16_t));
        }

        // read ADC
        int new_val[2];
        for(int i = 0; i < 2; i++)
        {
            ADC1->CR = ADC_CR_ADEN; // needing to re-enable seems strange?
            ADC1->CR = ADC_CR_ADSTART;

            while(!(ADC1->ISR & ADC_ISR_EOC))
                asm volatile("wfe");

            ADC1->ISR = ADC_ISR_EOC;

            new_val[i] = ADC1->DR;
        }

        // apply calibration scale
        new_val[0] -= calib_min[0];
        new_val[0] = (new_val[0] << 12) / (calib_max[0] - calib_min[0]);
        new_val[0] = std::max(0, std::min((1 << 12) - 1, new_val[0]));

        new_val[1] -= calib_min[1];
        new_val[1] = (new_val[1] << 12) / (calib_max[1] - calib_min[1]);
        new_val[1] = std::max(0, std::min((1 << 12) - 1, new_val[1]));

        // store new value
        adc_val[0] = new_val[0];
        adc_val[1] = new_val[1];

        auto new_inputs = gpio_get(GPIOA);
#ifndef BROKEN_V1
        // first button moved to B6
        new_inputs = (new_inputs & ~1) | ((gpio_get(GPIOB) >> 6) & 1);
#endif

        inputs = new_inputs;

        // merge adc+inputs into a single word
        *(uint32_t *)i2c_read_data = adc_val[0] | adc_val[1] << 16 | (inputs & 0xF) << 12 | (inputs & 0xF0) << 24;

        // LED update
        uint16_t r = led_gamma_10[i2c_write_data[0]];
        uint16_t g = led_gamma_10[i2c_write_data[1]];
        uint16_t b = led_gamma_10[i2c_write_data[2]];

#ifdef BROKEN_V1
        TIM1->CCR1 = g;
        // hack so R/B do something
        gpio_put(GPIOB, 2, b < 512);
        gpio_put(GPIOA, 5, r < 512);
#else
        TIM1->CCR1 = b;
        TIM1->CCR2 = r;
        TIM1->CCR3 = g;
#endif

        delay_ms(10);
    }

    return 0;
}