#ifndef __ZFOC_MAIN_H
#define __ZFOC_MAIN_H

// Hardware configuration
#include <board.h>

#include <math.h>

#ifdef __cplusplus
#include <task_timer.hpp>
 extern "C" {
#endif

#include <cmsis_os.h>

extern uint32_t _reboot_cookie;

extern uint64_t serial_number;
extern char serial_number_str[13];

#ifdef __cplusplus
}

typedef struct {
    bool fully_booted;
    uint32_t uptime; // [ms]
    uint32_t min_heap_space; // FreeRTOS heap [Bytes]
    uint32_t max_stack_usage_axis; // minimum remaining space since startup [Bytes]
    uint32_t max_stack_usage_startup;
    uint32_t max_stack_usage_can_a;
    uint32_t max_stack_usage_can_b;

    uint32_t stack_size_axis;
    uint32_t stack_size_startup;
    uint32_t stack_size_can_a;
    uint32_t stack_size_can_b;
    uint32_t stack_size_uart;

    int32_t prio_axis;
    int32_t prio_startup;
    int32_t prio_can_a;
    int32_t prio_can_b;
    int32_t prio_uart;
} SystemStats_t;

// @brief general user configurable board configuration
struct BoardConfig_t {

    bool enable_uart = true;
    uint32_t uart_baudrate = 115200;
    bool enable_can_a = true;
    bool enable_can_b = false;
    float max_regen_current = 0.0f;
    float brake_resistance = DEFAULT_BRAKE_RESISTANCE;
    bool enable_brake_resistor = false;
    float dc_bus_undervoltage_trip_level = DEFAULT_MIN_DC_VOLTAGE;      //<! [V] minimum voltage below which the motor stops operating
    float dc_bus_overvoltage_trip_level = 1.07f * HW_VERSION_VOLTAGE;   //<! [V] maximum voltage above which the motor stops operating.
                                                                        //<! This protects against cases in which the power supply fails to dissipate
                                                                        //<! the brake power if the brake resistor is disabled.
                                                                        //<! The default is 26V for the 24V board version and 52V for the 48V board version.

    /**
     * If enabled, if the measured DC voltage exceeds `dc_bus_overvoltage_ramp_start`,
     * the ODrive will sink more power than usual into the the brake resistor
     * in an attempt to bring the voltage down again.
     * 
     * The brake duty cycle is increased by the following amount:
     *  vbus_voltage == dc_bus_overvoltage_ramp_start  =>  brake_duty_cycle += 0%
     *  vbus_voltage == dc_bus_overvoltage_ramp_end  =>  brake_duty_cycle += 100%
     * 
     * Remarks:
     *  - This feature is active even when all motors are disarmed.
     *  - This feature is disabled if `brake_resistance` is non-positive.
     */
    bool enable_dc_bus_overvoltage_ramp = false;
    float dc_bus_overvoltage_ramp_start = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                      //!< Do not set this lower than your usual vbus_voltage,
                                                                      //!< unless you like fried brake resistors.
    float dc_bus_overvoltage_ramp_end = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                    //!< Must be larger than `dc_bus_overvoltage_ramp_start`,
                                                                    //!< otherwise the ramp feature is disabled.

    float dc_max_positive_current = INFINITY; // Max current [A] the power supply can source
    float dc_max_negative_current = -10.0f; // Max current [A] the power supply can sink. You most likely want a non-positive value here. Set to -INFINITY to disable.
};

struct TaskTimes {
    TaskTimer sampling;
    TaskTimer control_loop_misc;
    TaskTimer control_loop_checks;
    TaskTimer dc_calib_wait;
};

// Forward Declarations
class Axis;
class Motor;

// TODO: move
// this is technically not thread-safe but practically it might be
#define DEFINE_ENUM_FLAG_OPERATORS(ENUMTYPE) \
inline ENUMTYPE operator | (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) | static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator & (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) & static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator ^ (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) ^ static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator |= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) |= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator &= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) &= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator ^= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) ^= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator ~ (ENUMTYPE a) { return static_cast<ENUMTYPE>(~static_cast<std::underlying_type_t<ENUMTYPE>>(a)); }

#include "interfaces.hpp"

// ODrive specific includes
#include <utils.hpp>
#include <low_level.h>
#include <encoder.hpp>
#include <controller.hpp>
#include <current_limiter.hpp>
#include <trapTraj.hpp>
#include <axis.hpp>
#include <communication/communication.h>
#include <communication/can/zfoc_can.hpp>

// Defined in autogen/version.c based on git-derived version numbers
extern "C" {
extern const unsigned char fw_version_major_;
extern const unsigned char fw_version_minor_;
extern const unsigned char fw_version_revision_;
extern const unsigned char fw_version_unreleased_;
}

// general system functions defined in main.cpp
class Zfoc : public ZfocIntf {
public:
    bool save_configuration();
    void erase_configuration();
    void reboot() { NVIC_SystemReset(); }
    void clear_errors();

    bool any_error();
    void update_calibration_save_state();

    void do_fast_checks();
    void sampling_cb();
    void control_loop_cb(uint32_t timestamp);

    Axis& get_axis(int num) { return axes[num]; }

    uint32_t get_interrupt_status(int32_t irqn);
    void disarm_with_error(Error error);

    Error error_ = ERROR_NONE;
    float& vbus_voltage_ = ::vbus_voltage; // TODO: make this the actual variable
    float& ibus_ = ::ibus_; // TODO: make this the actual variable
    float ibus_report_filter_k_ = 1.0f;

    const uint64_t& serial_number_ = ::serial_number;

    // Hardware version is compared with OTP on startup to ensure that we're
    // running on the right board version.
    const uint8_t hw_version_major_ = HW_VERSION_MAJOR;
    const uint8_t hw_version_minor_ = HW_VERSION_MINOR;
    const uint8_t hw_version_variant_ = HW_VERSION_VOLTAGE;

    // the corresponding macros are defined in the autogenerated version.h
    const uint8_t fw_version_major_ = ::fw_version_major_;
    const uint8_t fw_version_minor_ = ::fw_version_minor_;
    const uint8_t fw_version_revision_ = ::fw_version_revision_;
    const uint8_t fw_version_unreleased_ = ::fw_version_unreleased_; // 0 for official releases, 1 otherwise

    bool& brake_resistor_armed_ = ::brake_resistor_armed; // TODO: make this the actual variable
    bool& brake_resistor_saturated_ = ::brake_resistor_saturated; // TODO: make this the actual variable
    float& brake_resistor_current_ = ::brake_resistor_current;

    SystemStats_t system_stats_;

    ZfocCAN can_a;
    ZfocCAN can_b;

    BoardConfig_t config_;
    uint32_t user_config_loaded_ = 0;
    bool config_loaded_from_nvm_ = false;
    bool calibration_save_pending_ = false;
    bool config_save_in_progress_ = false;
    bool misconfigured_ = false;

    uint32_t test_property_ = 0;

    uint32_t last_update_timestamp_ = 0;
    uint32_t n_evt_sampling_ = 0;
    uint32_t n_evt_control_loop_ = 0;
    bool task_timers_armed_ = false;
    TaskTimes task_times_;
};

extern Zfoc zfoc; // defined in main.cpp

#endif // __cplusplus

#endif //__ZFOC_MAIN_H
