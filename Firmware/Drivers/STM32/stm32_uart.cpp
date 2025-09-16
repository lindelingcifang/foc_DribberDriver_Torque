#include <stm32_uart.hpp>
#include <stm32_system.h>
#include <utils.hpp>
#include <cmsis_os.h>

bool Stm32Uart::acquire_task(UartTask* task) {
    return !__atomic_exchange_n(&task->is_in_use, true, __ATOMIC_SEQ_CST);
}

void Stm32Uart::release_task(UartTask* task) {
    task->is_in_use = false;
}

bool Stm32Uart::start() {
    if (!task_list_) {
        return false;
    }

    UartTask& task = *task_list_;
    // Reinit UART if the Baudrate doesn't match
    if (task.baudrate != huart_->Init.BaudRate) {
        HAL_UART_DeInit(huart_);
        huart_->Init.BaudRate = task.baudrate;
        HAL_UART_Init(huart_);
        __HAL_UART_ENABLE(huart_);
    }

    HAL_StatusTypeDef status = HAL_ERROR;

    if (huart_->hdmatx->State != HAL_DMA_STATE_READY || huart_->hdmarx->State != HAL_DMA_STATE_READY) {
        // This can happen if the DMA or interrupt priorities are not configured properly.
        status = HAL_BUSY;
    } else if (task.tx_buf && task.rx_buf) {
        status = HAL_UART_Transmit_DMA(huart_, (uint8_t*)task.tx_buf, task.length);
        status = HAL_UART_Receive_DMA(huart_, (uint8_t*)task.rx_buf, task.length);
    } else if (task.tx_buf) {
        status = HAL_UART_Transmit_DMA(huart_, (uint8_t*)task.tx_buf, task.length);
    } else if (task.rx_buf) {
        status = HAL_UART_Receive_DMA(huart_, (uint8_t*)task.rx_buf, task.length);
    }

    return status == HAL_OK;
}

bool Stm32Uart::transfer_async(UartTask* task) {
    task->next = nullptr;
    
    // Append new task to task list.
    // We could try to do this lock free but we could also use our time for useful things.
    UartTask** ptr = &task_list_;
    CRITICAL_SECTION() {
        while (*ptr)
            ptr = &(*ptr)->next;
        *ptr = task;
    }

    // If the list was empty before, kick off the UART now
    if (ptr == &task_list_) {
        if (!start()) {
            if (task->on_complete) {
                (*task->on_complete)(task->on_complete_ctx, false);
            }
        }
    }
}

void Stm32Uart::on_complete() {
    if (!task_list_) {
        return; // this should not happen
    }

    // Wrap up transfer
    if (task_list_->on_complete) {
        (*task_list_->on_complete)(task_list_->on_complete_ctx, true);
    }

    // Start next task if any
    UartTask* next = nullptr;
    CRITICAL_SECTION() {
        next = task_list_ = task_list_->next;
    }
    if (next) {
        start();
    }
}
