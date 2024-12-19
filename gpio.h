#pragma once
#include "py32f0xx.h"

enum gpio_mode
{
    GPIO_MODE_INPUT     = 0,
    GPIO_MODE_OUTPUT    = 1,
    GPIO_MODE_ALTERNATE = 2,
    GPIO_MODE_ANALOG    = 3,
};

enum gpio_output_type
{
    GPIO_OUTPUT_PUSH_PULL  = 0,
    GPIO_OUTPUT_OPEN_DRAIN = 1,
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

inline void gpio_set_output_type(GPIO_TypeDef *port, int pin, int otype)
{
    port->OTYPER = (port->OTYPER & ~(GPIO_OTYPER_OT0 << pin))
                 | otype << pin;
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

inline void gpio_put(GPIO_TypeDef *port, int pin, bool high)
{
    if(high)
        port->BSRR = 1 << pin;
    else
        port->BSRR = 1 << (pin + 16);
}