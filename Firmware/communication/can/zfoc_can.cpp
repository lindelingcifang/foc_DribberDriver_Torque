#include "zfoc_can.hpp"

#include <fdcan.h>
#include <cmsis_os.h>

#include "freertos_vars.h"
#include "utils.hpp"

#include "fdcan.h"

// Safer context handling via maps instead of arrays
// #include <unordered_map>
// std::unordered_map<CAN_HandleTypeDef *, ZfocCAN *> ctxMap;


bool ZfocCAN::apply_config() {
    config_.parent = this;
    set_baud_rate(config_.baud_rate);
    return true;
}

HAL_StatusTypeDef FDCAN_ResetError(FDCAN_HandleTypeDef *hfdcan) {
    HAL_StatusTypeDef status = HAL_OK;

    if ((hfdcan->State == HAL_FDCAN_STATE_READY) || 
    (hfdcan->State == HAL_FDCAN_STATE_BUSY)) {
        // Reset Error Code
        hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
    } else {
        hfdcan->ErrorCode |= HAL_FDCAN_ERROR_NOT_INITIALIZED;
        status = HAL_ERROR;
    }

    return status;
}

bool ZfocCAN::reinit() {
    HAL_FDCAN_Stop(handle_);
    FDCAN_ResetError(handle_);
    return (HAL_FDCAN_Init(handle_) == HAL_OK)
        // && (HAL_FDCAN_ConfigFilter(handle_, &filter_) == HAL_OK)
        && (HAL_FDCAN_Start(handle_) == HAL_OK)
        && (HAL_FDCAN_ActivateNotification(handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | 
            // FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL | 
            FDCAN_IT_TX_FIFO_EMPTY, 0) == HAL_OK);
}

bool ZfocCAN::start_server(FDCAN_HandleTypeDef* handle, FDCAN_GlobalTypeDef* instance) {
    handle_ = handle;

    handle_->Instance = instance;
    handle_->Init.ClockDivider = FDCAN_CLOCK_DIV1;
    handle_->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    handle_->Init.Mode = FDCAN_MODE_NORMAL;
    handle_->Init.AutoRetransmission = DISABLE;
    handle_->Init.TransmitPause = DISABLE;
    handle_->Init.ProtocolException = DISABLE;
    handle_->Init.NominalPrescaler = CAN_FREQ / config_.baud_rate;
    handle_->Init.NominalSyncJumpWidth = 1;
    handle_->Init.NominalTimeSeg1 = CAN_PBS1;
    handle_->Init.NominalTimeSeg2 = CAN_PBS2;
    handle_->Init.DataPrescaler = 1;
    handle_->Init.DataSyncJumpWidth = 1;
    handle_->Init.DataTimeSeg1 = 1;
    handle_->Init.DataTimeSeg2 = 1;
    handle_->Init.StdFiltersNbr = 28;
    handle_->Init.ExtFiltersNbr = 0;
    handle_->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    // filter_.IdType = FDCAN_STANDARD_ID;
    // filter_.FilterIndex = 0;
    // filter_.FilterType = FDCAN_FILTER_RANGE;
    // filter_.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // filter_.FilterID1 = 0;
    // filter_.FilterID2 = 0;
    if (!reinit()) {
        return false;
    }

    auto wrapper = [](const void* ctx) {
        ((ZfocCAN*)ctx)->can_server_thread();
    };
    osThreadDef(can_server_thread_def, wrapper, osPriorityNormal, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(can_server_thread_def), this);

    return true;
}

void ZfocCAN::can_server_thread() {
    Protocol protocol = config_.protocol;

    if (protocol & PROTOCOL_SIMPLE) {
        can_simple_.init();
    }

    for (;;) {
        uint32_t status = HAL_FDCAN_GetError(handle_);
        if (status == HAL_FDCAN_ERROR_NONE) {
            uint32_t next_service_time = UINT32_MAX;

            if (protocol & PROTOCOL_SIMPLE) {
                next_service_time = std::min(can_simple_.service_stack(), next_service_time);
            }

            process_rx_fifo(FDCAN_RX_FIFO0);
            process_rx_fifo(FDCAN_RX_FIFO1);
            HAL_FDCAN_ActivateNotification(handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0);

            // wait at least 1ms to prevent busy-spin on failed sends
            osSemaphoreWait(sem_can, std::max(next_service_time, 1UL));
        } else if (status == HAL_FDCAN_ERROR_TIMEOUT) {
            FDCAN_ResetError(handle_);
            status = HAL_FDCAN_Start(handle_);
            if (status == HAL_OK)
                status = HAL_FDCAN_ActivateNotification(handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0);
        }
    }
}

// Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
bool ZfocCAN::set_baud_rate(uint32_t baud_rate) {
    uint32_t prescaler = CAN_FREQ / baud_rate;
    if (prescaler * baud_rate == CAN_FREQ) {
        // valid baud rate
        config_.baud_rate = baud_rate;
        if (handle_) {
            handle_->Init.NominalPrescaler = prescaler;
            return reinit();
        }
        return true;
    } else {
        // invalid baud rate - ignore
        return false;
    }
}

void ZfocCAN::process_rx_fifo(uint32_t fifo) {
    while (HAL_FDCAN_GetRxFifoFillLevel(handle_, fifo)) {
        volatile uint32_t fill_level = HAL_FDCAN_GetRxFifoFillLevel(handle_, fifo);
        FDCAN_RxHeaderTypeDef header;
        can_Message_t rxmsg;
        HAL_FDCAN_GetRxMessage(handle_, fifo, &header, rxmsg.buf);

        rxmsg.isExt = header.IdType == FDCAN_EXTENDED_ID;
        rxmsg.id = header.Identifier;  // either standard (11-bit) or extended (29-bit) ID
        rxmsg.len = header.DataLength;
        rxmsg.rtr = header.RxFrameType == FDCAN_REMOTE_FRAME;

        // TODO: this could be optimized with an ahead-of-time computed
        // index-to-filter map

        size_t fifo0_idx = 0;
        size_t fifo1_idx = 0;

        // Find the triggered subscription item based on header.FilterMatchIndex
        auto it = std::find_if(subscriptions_.begin(), subscriptions_.end(), [&](auto& s) {
            size_t current_idx = (s.fifo == FDCAN_RX_FIFO0 ? fifo0_idx : fifo1_idx)++;
            return (header.FilterIndex == current_idx) && (s.fifo == fifo);
        });

        if (it == subscriptions_.end()) {
            continue;
        }

        it->callback(it->ctx, rxmsg);
    }
}

// Send a CAN message on the bus
bool ZfocCAN::send_message(const can_Message_t &txmsg) {
    if (HAL_FDCAN_GetError(handle_) != HAL_FDCAN_ERROR_NONE) {
        return false;
    }

    FDCAN_TxHeaderTypeDef header;
    header.Identifier = txmsg.id;
    header.IdType = txmsg.isExt ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = txmsg.len;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    volatile uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(handle_);

    if (!HAL_FDCAN_GetTxFifoFreeLevel(handle_)) {
        return false;
    }
    
    return HAL_FDCAN_AddMessageToTxFifoQ(handle_, &header, (uint8_t*)txmsg.buf) == HAL_OK;
}

//void ZfocCAN::set_error(Error error) {
//    error_ |= error;
//}

bool ZfocCAN::subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) {
    auto it = std::find_if(subscriptions_.begin(), subscriptions_.end(), [](auto& subscription) {
        return subscription.fifo == kCanFifoNone;
    });

    if (it == subscriptions_.end()) {
        return false; // all subscription slots in use
    }

    it->callback = callback;
    it->ctx = ctx;
    it->fifo = FDCAN_RX_FIFO0; // TODO: make customizable
    if (handle) {
        *handle = &*it;
    }

    bool is_extended = filter.id.index() == 1;
    uint32_t id = is_extended ? (std::get<1>(filter.id) & 0x1FFFFFFF) : (std::get<0>(filter.id) & 0x7FF);
    uint32_t mask = is_extended ? (filter.mask & 0x1FFFFFFF) : (filter.mask & 0x7FF);

    FDCAN_FilterTypeDef hal_filter;
    hal_filter.IdType = is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    hal_filter.FilterIndex = &*it - &subscriptions_[0];
    hal_filter.FilterType = FDCAN_FILTER_MASK;
    hal_filter.FilterConfig = (it->fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    hal_filter.FilterID1 = id;
    hal_filter.FilterID2 = mask;

    if (HAL_FDCAN_ConfigFilter(handle_, &hal_filter) != HAL_OK) {
        return false;
    }
    return true;
}

bool ZfocCAN::unsubscribe(CanSubscription* handle) {
    ZfocCANSubscription* subscription = static_cast<ZfocCANSubscription*>(handle);
    if (subscription < subscriptions_.begin() || subscription >= subscriptions_.end()) {
        return false;
    }
    if (subscription->fifo != kCanFifoNone) {
        return false; // not in use
    }

    subscription->fifo = kCanFifoNone;

    FDCAN_FilterTypeDef hal_filter = {};
    hal_filter.FilterConfig = FDCAN_FILTER_DISABLE;
    return HAL_FDCAN_ConfigFilter(handle_, &hal_filter) == HAL_OK;
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
    HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY);
    osSemaphoreRelease(sem_can);
}
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan) {}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
        HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        osSemaphoreRelease(sem_can);
    } else if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) {
        // FIFO full - should never happen with our low traffic, but just in case
        HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_FULL);
        osSemaphoreRelease(sem_can);
    }
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if (!(RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {
        HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
        osSemaphoreRelease(sem_can);
    } else if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL) {
        // FIFO full - should never happen with our low traffic, but just in case
        HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_FULL);
        osSemaphoreRelease(sem_can);
    }
}
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
    //HAL_CAN_ResetError(hcan);
}
