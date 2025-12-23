#ifndef __STM32_UART_HPP
#define __STM32_UART_HPP

#include <stm32g474xx.h>
#include <stm32g4xx_hal_uart.h>

class Stm32Uart {
public:
    static const Stm32Uart none;
    Stm32Uart() : huart_(nullptr) {}
    Stm32Uart(UART_HandleTypeDef* uart) : huart_(uart) {}

    operator bool() const { return huart_ && huart_->Instance; }

    struct UartTask {
        uint32_t baudrate;
        const uint8_t* tx_buf;
        uint8_t* rx_buf;
        size_t length;
        void (*on_complete)(void*, bool);
        void* on_complete_ctx;
        bool is_in_use = false;
        struct UartTask* next = nullptr;
    };

    /**
     * Reserves the task for the caller if it's not in use currently.
     *
     * This can be used by the caller to ensure that the task structure is not
     * overwritten while it's in use in a preceding transfer.
     *
     * Example:
     *
     *     if (acquire_task(&task)) {
     *         transfer_async(&task)
     *     }
     *
     * A call to release_task() makes the task available for use again.
     */
    static bool acquire_task(UartTask* task);
    /**
     * Releases the task so that the next call to `acquire_task()` returns true.
     * This should usually be called inside the on_complete() callback after
     * the rx buffer has been processed.
     */
    static void release_task(UartTask* task);

    /**
     * @brief Enqueues a non-blocking transfer.
     * 
     * Once the transfer completes, fails or is aborted, the callback is invoked.
     * 
     * This function is thread-safe with respect to all other public functions
     * of this class.
     * 
     * @param task: Contains all configuration data for this transfer.
     *        The struct pointed to by this argument must remain valid and
     *        unmodified until the completion callback is invoked.
     */
    bool transfer_async(UartTask* task);
    /**
     * @brief Completion method to be called from
     * HAL_UART_TxCpltCallback and HAL_UART_RxCpltCallback.
     */
    void on_complete();
private:
    bool start();
    UART_HandleTypeDef* huart_;
    UartTask* task_list_ = nullptr;
};

#endif