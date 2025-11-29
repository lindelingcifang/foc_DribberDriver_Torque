#include "stm32_i2c.hpp"
#include "stm32_system.h"
    
bool Stm32I2c::acquire_task(I2cTask* task) {
    return !__atomic_exchange_n(&task->is_in_use, true, __ATOMIC_SEQ_CST);
}

void Stm32I2c::release_task(I2cTask* task) {
    task->is_in_use = false;
}

bool Stm32I2c::start() {
    if (!task_list_) {
        return false;
    }

    I2cTask& task = *task_list_;

    HAL_StatusTypeDef status = HAL_ERROR;

    if (task.reg_addr) {
        if (task.tx_buf && task.rx_buf) {
            status = HAL_I2C_Mem_Write_DMA(hi2c_, task.dev_addr << 1, task.reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)task.tx_buf, task.length);
            status = HAL_I2C_Mem_Read_DMA(hi2c_, task.dev_addr << 1, task.reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)task.rx_buf, task.length);
        } else if (task.tx_buf) {
            status = HAL_I2C_Mem_Write_DMA(hi2c_, task.dev_addr << 1, task.reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)task.tx_buf, task.length);
        } else if (task.rx_buf) {
            status = HAL_I2C_Mem_Read_DMA(hi2c_, task.dev_addr << 1, task.reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)task.rx_buf, task.length);
        }
    } else {
        if (task.tx_buf && task.rx_buf) {
            status = HAL_I2C_Master_Transmit_DMA(hi2c_, task.dev_addr << 1, (uint8_t*)task.tx_buf, task.length);
            status = HAL_I2C_Master_Receive_DMA(hi2c_, task.dev_addr << 1, (uint8_t*)task.rx_buf, task.length);
        } else if (task.tx_buf) {
            status = HAL_I2C_Master_Transmit_DMA(hi2c_, task.dev_addr << 1, (uint8_t*)task.tx_buf, task.length);
        } else if (task.rx_buf) {
            status = HAL_I2C_Master_Receive_DMA(hi2c_, task.dev_addr << 1, (uint8_t*)task.rx_buf, task.length);
        }
    }
    return status == HAL_OK;
}

bool Stm32I2c::transfer_async(I2cTask* task) {
    task->next = nullptr;
    
    // Append new task to task list.
    // We could try to do this lock free but we could also use our time for useful things.
    I2cTask** ptr = &task_list_;
    CRITICAL_SECTION() {
        while (*ptr)
            ptr = &(*ptr)->next;
        *ptr = task;
    }

    // If the list was empty before, kick off the I2C now
    if (ptr == &task_list_) {
        if (!start()) {
            if (task->on_complete) {
                task->on_complete(task->on_complete_ctx, false);
            }
        }
    }

    return true;
}

void Stm32I2c::on_complete() {
    if (!task_list_) {
        return; // this should not happen
    }

    // Wrap up transfer
    if (task_list_->on_complete) {
        task_list_->on_complete(task_list_->on_complete_ctx, true);
    }

    // Start next task if any
    I2cTask* next = nullptr;
    CRITICAL_SECTION() {
        next = task_list_ = task_list_->next;
    }
    if (next) {
        start();
    }
}

