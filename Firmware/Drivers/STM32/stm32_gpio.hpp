#ifndef __STM32_GPIO_HPP
#define __STM32_GPIO_HPP

#include <gpio.h>
#include <stm32g474xx.h>
#include <stm32g4xx_hal_gpio.h>

class Stm32Gpio {
public:
    static const Stm32Gpio none;

    Stm32Gpio() : port_(nullptr), pin_mask_(0) {}
    Stm32Gpio(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_mask_(pin) {}

    operator bool() const { return port_ && pin_mask_; }

    /**
     * @brief Configures the GPIO with the specified parameters.
     * 
     * This can be done regardless of the current state of the GPIO.
     * 
     * If any subscription is in place, it is not disabled by this function.
     */
    bool config(uint32_t mode, uint32_t pull, uint32_t speed = GPIO_SPEED_FREQ_LOW);

    void write(bool state) {
        if (port_) {
            HAL_GPIO_WritePin(port_, pin_mask_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }

    bool read() {
        return port_ && (port_->IDR & pin_mask_);
    }

    uint16_t get_pin_number() {
        uint16_t pin_number = 0;
        uint16_t pin_mask = pin_mask_ >> 1;
        while (pin_mask) {
            pin_mask >>= 1;
            pin_number++;
        }
        return pin_number;
    }

    GPIO_TypeDef* port_;
    uint16_t pin_mask_; // TODO: store pin_number_ instead of pin_mask_
};

#endif // __STM32_GPIO_HPP
