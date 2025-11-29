#ifndef STM32_I2C_HPP
#define STM32_I2C_HPP

#include "i2c.h"
#include <stm32g4xx_hal_i2c.h>

class Stm32I2c {
public:
    static const Stm32I2c none;
    Stm32I2c() : hi2c_(nullptr) {}
    Stm32I2c(I2C_HandleTypeDef* i2c) : hi2c_(i2c) {}

    struct I2cTask {
        uint8_t dev_addr;
        uint8_t reg_addr;
        const uint8_t* tx_buf;
        uint8_t* rx_buf;
        size_t length;
        void (*on_complete)(void*, bool);
        void* on_complete_ctx;
        bool is_in_use = false;
        struct I2cTask* next = nullptr;
    };

    static bool acquire_task(I2cTask* task);
    static void release_task(I2cTask* task);
    bool transfer_async(I2cTask* task);
    void on_complete();

private:
    bool start();
    I2C_HandleTypeDef* hi2c_;
    I2cTask* task_list_ = nullptr;
};
#endif