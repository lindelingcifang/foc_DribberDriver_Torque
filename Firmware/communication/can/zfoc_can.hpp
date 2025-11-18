#ifndef __ODRIVE_CAN_HPP
#define __ODRIVE_CAN_HPP

#include <cmsis_os.h>

#include "canbus.hpp"
#include "can_simple.hpp"
#include <interfaces.hpp>

#define CAN_CLK_HZ (170000000)
#define CAN_CLK_MHZ (170)
#define CAN_PBS1 (6)
#define CAN_PBS2 (3)
#define CAN_TQ (1 + CAN_PBS1 + CAN_PBS2)
#define CAN_FREQ (CAN_CLK_HZ / CAN_TQ)  // (CAN_FREQ / Prescaler): 10MHz for 1Mbit/s, 5MHz for 500kbit/s, etc.

// Anonymous enum for defining the most common CAN baud rates
enum {
    CAN_BAUD_125K = 125000,
    CAN_BAUD_250K = 250000,
    CAN_BAUD_500K = 500000,
    CAN_BAUD_1000K = 1000000,
    CAN_BAUD_1M = 1000000
};

class ZfocCAN : public CanBusBase, public ZfocIntf::CanIntf {
public:
    struct Config_t {
        uint32_t baud_rate = CAN_BAUD_1M;
        Protocol protocol = PROTOCOL_SIMPLE;

        ZfocCAN* parent = nullptr; // set in apply_config()
        void set_baud_rate(uint32_t value) { parent->set_baud_rate(value); }
    };

    ZfocCAN() {}

    bool apply_config();
    bool start_server(FDCAN_HandleTypeDef* handle, FDCAN_GlobalTypeDef* instance);

    Error error_ = ERROR_NONE;

    Config_t config_;
    CANSimple can_simple_{this};

    osThreadId thread_id_;
    const uint32_t stack_size_ = 1024;  // Bytes

private:
    static const uint8_t kCanFifoNone = 0xff;

    struct ZfocCANSubscription : CanSubscription {
        uint8_t fifo = kCanFifoNone;
        on_can_message_cb_t callback;
        void* ctx;
    };

    bool reinit();
    void can_server_thread();
    bool set_baud_rate(uint32_t baud_rate);
    void process_rx_fifo(uint32_t fifo);
    bool send_message(const can_Message_t& message) final;
    bool subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) final;
    bool unsubscribe(CanSubscription* handle) final;

    // Hardware supports at most 28 filters unless we do optimizations. For now
    // we don't need that many.
    std::array<ZfocCANSubscription, 8> subscriptions_;
    FDCAN_HandleTypeDef *handle_ = nullptr;
    FDCAN_FilterTypeDef filter_ = {};
};

#endif  // __ODRIVE_CAN_HPP
