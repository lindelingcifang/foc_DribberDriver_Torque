#ifndef __INTERFACES_HPP
#define __INTERFACES_HPP

#include <cstdint>

#include <tuple>
using float2D = std::pair<float, float>;
struct Iph_ABC_t { float phA; float phB; float phC; };

#pragma GCC push_options
#pragma GCC optimize ("s")


class ZfocIntf {
public:
    class CanIntf {
    public:
        enum Error {
            ERROR_NONE                       = 0x00000000,
            ERROR_DUPLICATE_CAN_IDS          = 0x00000001,
        };
        enum Protocol {
            PROTOCOL_SIMPLE                  = 0x00000001,
        };
    };
    class AxisIntf {
    public:
        enum Error {
            ERROR_NONE                       = 0x00000000,
            ERROR_INVALID_STATE              = 0x00000001,
            ERROR_MOTOR_FAILED               = 0x00000040,
            ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x00000080,
            ERROR_ENCODER_FAILED             = 0x00000100,
            ERROR_CONTROLLER_FAILED          = 0x00000200,
            ERROR_WATCHDOG_TIMER_EXPIRED     = 0x00000800,
            ERROR_MIN_ENDSTOP_PRESSED        = 0x00001000,
            ERROR_MAX_ENDSTOP_PRESSED        = 0x00002000,
            ERROR_ESTOP_REQUESTED            = 0x00004000,
            ERROR_HOMING_WITHOUT_ENDSTOP     = 0x00020000,
            ERROR_OVER_TEMP                  = 0x00040000,
            ERROR_UNKNOWN_POSITION           = 0x00080000,
        };
        enum AxisState {
            AXIS_STATE_UNDEFINED             = 0,
            AXIS_STATE_IDLE                  = 1,
            AXIS_STATE_STARTUP_SEQUENCE      = 2,
            AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
            AXIS_STATE_MOTOR_CALIBRATION     = 4,
            AXIS_STATE_ENCODER_INDEX_SEARCH  = 6,
            AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
            AXIS_STATE_CLOSED_LOOP_CONTROL   = 8,
            AXIS_STATE_LOCKIN_SPIN           = 9,
            AXIS_STATE_ENCODER_DIR_FIND      = 10,
            AXIS_STATE_HOMING                = 11,
            AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
            AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
        };
        virtual void watchdog_feed() = 0;
    };
    class MotorIntf {
    public:
        enum Error {
            ERROR_NONE                       = 0x00000000,
            ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001,
            ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002,
            ERROR_DRV_FAULT                  = 0x00000008,
            ERROR_CONTROL_DEADLINE_MISSED    = 0x00000010,
            ERROR_MODULATION_MAGNITUDE       = 0x00000080,
            ERROR_CURRENT_SENSE_SATURATION   = 0x00000400,
            ERROR_CURRENT_LIMIT_VIOLATION    = 0x00001000,
            ERROR_MODULATION_IS_NAN          = 0x00010000,
            ERROR_MOTOR_THERMISTOR_OVER_TEMP = 0x00020000,
            ERROR_FET_THERMISTOR_OVER_TEMP   = 0x00040000,
            ERROR_TIMER_UPDATE_MISSED        = 0x00080000,
            ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = 0x00100000,
            ERROR_CONTROLLER_FAILED          = 0x00200000,
            ERROR_I_BUS_OUT_OF_RANGE         = 0x00400000,
            ERROR_BRAKE_RESISTOR_DISARMED    = 0x00800000,
            ERROR_SYSTEM_LEVEL               = 0x01000000,
            ERROR_BAD_TIMING                 = 0x02000000,
            ERROR_UNKNOWN_PHASE_ESTIMATE     = 0x04000000,
            ERROR_UNKNOWN_PHASE_VEL          = 0x08000000,
            ERROR_UNKNOWN_TORQUE             = 0x10000000,
            ERROR_UNKNOWN_CURRENT_COMMAND    = 0x20000000,
            ERROR_UNKNOWN_CURRENT_MEASUREMENT = 0x40000000,
            ERROR_UNKNOWN_VBUS_VOLTAGE       = 0x80000000,
            ERROR_UNKNOWN_VOLTAGE_COMMAND    = 0x100000000,
            ERROR_UNKNOWN_GAINS              = 0x200000000,
            ERROR_CONTROLLER_INITIALIZING    = 0x400000000,
            ERROR_UNBALANCED_PHASES          = 0x800000000,
        };
        enum MotorType {
            MOTOR_TYPE_HIGH_CURRENT          = 0,
            MOTOR_TYPE_GIMBAL                = 2,
            MOTOR_TYPE_ACIM                  = 3,
        };
    };
    class ControllerIntf {
    public:
        enum Error {
            ERROR_NONE                       = 0x00000000,
            ERROR_OVERSPEED                  = 0x00000001,
            ERROR_INVALID_INPUT_MODE         = 0x00000002,
            ERROR_UNSTABLE_GAIN              = 0x00000004,
            ERROR_INVALID_MIRROR_AXIS        = 0x00000008,
            ERROR_INVALID_LOAD_ENCODER       = 0x00000010,
            ERROR_INVALID_ESTIMATE           = 0x00000020,
            ERROR_INVALID_CIRCULAR_RANGE     = 0x00000040,
            ERROR_SPINOUT_DETECTED           = 0x00000080,
        };
        enum ControlMode {
            CONTROL_MODE_VOLTAGE_CONTROL     = 0,
            CONTROL_MODE_TORQUE_CONTROL      = 1,
            CONTROL_MODE_VELOCITY_CONTROL    = 2,
            CONTROL_MODE_POSITION_CONTROL    = 3,
        };
        enum InputMode {
            INPUT_MODE_INACTIVE              = 0,
            INPUT_MODE_PASSTHROUGH           = 1,
            INPUT_MODE_VEL_RAMP              = 2,
            INPUT_MODE_POS_FILTER            = 3,
            INPUT_MODE_MIX_CHANNELS          = 4,
            INPUT_MODE_TRAP_TRAJ             = 5,
            INPUT_MODE_TORQUE_RAMP           = 6,
            INPUT_MODE_MIRROR                = 7,
            INPUT_MODE_TUNING                = 8,
        };
        virtual void move_incremental(float displacement, bool from_input_pos) = 0;
        virtual void start_anticogging_calibration() = 0;
        virtual float remove_anticogging_bias() = 0;
        virtual float get_anticogging_value(uint32_t index) = 0;
    };
    class EncoderIntf {
    public:
        enum Error {
            ERROR_NONE                       = 0x00000000,
            ERROR_UNSTABLE_GAIN              = 0x00000001,
            ERROR_CPR_POLEPAIRS_MISMATCH     = 0x00000002,
            ERROR_NO_RESPONSE                = 0x00000004,
            ERROR_UNSUPPORTED_ENCODER_MODE   = 0x00000008,
            ERROR_ILLEGAL_HALL_STATE         = 0x00000010,
            ERROR_INDEX_NOT_FOUND_YET        = 0x00000020,
            ERROR_ABS_SPI_COM_FAIL           = 0x00000040,
            ERROR_HALL_NOT_CALIBRATED_YET    = 0x00000200,
        };
        enum Mode {
            MODE_DISABLED                    = 0,
            MODE_SPI_ABS_MT6701              = 0x100,
            MODE_SPI_ABS_AS5047P             = 0x101,
        };
    };
    class TrapezoidalTrajectoryIntf {
    public:
    };
    class TaskTimerIntf {
    public:
    };
    class TaskTimesIntf {
    public:
    };
    class SystemStatsIntf {
    public:
    };
    enum Error {
        ERROR_NONE                       = 0x00000000,
        ERROR_CONTROL_ITERATION_MISSED   = 0x00000001,
        ERROR_DC_BUS_UNDER_VOLTAGE       = 0x00000002,
        ERROR_DC_BUS_OVER_VOLTAGE        = 0x00000004,
        ERROR_DC_BUS_OVER_REGEN_CURRENT  = 0x00000008,
        ERROR_DC_BUS_OVER_CURRENT        = 0x00000010,
        ERROR_BRAKE_DEADTIME_VIOLATION   = 0x00000020,
        ERROR_BRAKE_DUTY_CYCLE_NAN       = 0x00000040,
        ERROR_INVALID_BRAKE_RESISTANCE   = 0x00000080,
    };
    enum GpioMode {
        GPIO_MODE_DIGITAL                = 0,
        GPIO_MODE_DIGITAL_PULL_UP        = 1,
        GPIO_MODE_DIGITAL_PULL_DOWN      = 2,
        GPIO_MODE_ANALOG_IN              = 3,
        GPIO_MODE_UART                   = 4,
        GPIO_MODE_CAN_A                  = 5,
        GPIO_MODE_CAN_B                  = 6,
        GPIO_MODE_PWM                    = 7,
        GPIO_MODE_ENC                    = 8,
        GPIO_MODE_STATUS                 = 9,
    };
    enum StreamProtocolType {
        STREAM_PROTOCOL_TYPE_FIBRE       = 0,
        STREAM_PROTOCOL_TYPE_ASCII       = 1,
        STREAM_PROTOCOL_TYPE_STDOUT      = 2,
        STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT = 3,
    };
    virtual bool save_configuration() = 0;
    virtual void erase_configuration() = 0;
    virtual void reboot() = 0;
    virtual uint32_t get_interrupt_status(int32_t irqn) = 0;
    virtual void clear_errors() = 0;
};


// this is technically not thread-safe but practically it might be
inline ZfocIntf::CanIntf::Protocol operator | (ZfocIntf::CanIntf::Protocol a, ZfocIntf::CanIntf::Protocol b) { return static_cast<ZfocIntf::CanIntf::Protocol>(static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(a) | static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(b)); }
inline ZfocIntf::CanIntf::Protocol operator & (ZfocIntf::CanIntf::Protocol a, ZfocIntf::CanIntf::Protocol b) { return static_cast<ZfocIntf::CanIntf::Protocol>(static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(a) & static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(b)); }
inline ZfocIntf::CanIntf::Protocol operator ^ (ZfocIntf::CanIntf::Protocol a, ZfocIntf::CanIntf::Protocol b) { return static_cast<ZfocIntf::CanIntf::Protocol>(static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(a) ^ static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(b)); }
inline ZfocIntf::CanIntf::Protocol& operator |= (ZfocIntf::CanIntf::Protocol &a, ZfocIntf::CanIntf::Protocol b) { return reinterpret_cast<ZfocIntf::CanIntf::Protocol&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>&>(a) |= static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(b)); }
inline ZfocIntf::CanIntf::Protocol& operator &= (ZfocIntf::CanIntf::Protocol &a, ZfocIntf::CanIntf::Protocol b) { return reinterpret_cast<ZfocIntf::CanIntf::Protocol&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>&>(a) &= static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(b)); }
inline ZfocIntf::CanIntf::Protocol& operator ^= (ZfocIntf::CanIntf::Protocol &a, ZfocIntf::CanIntf::Protocol b) { return reinterpret_cast<ZfocIntf::CanIntf::Protocol&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>&>(a) ^= static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(b)); }
inline ZfocIntf::CanIntf::Protocol operator ~ (ZfocIntf::CanIntf::Protocol a) { return static_cast<ZfocIntf::CanIntf::Protocol>(~static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Protocol>>(a)); }
// this is technically not thread-safe but practically it might be
inline ZfocIntf::Error operator | (ZfocIntf::Error a, ZfocIntf::Error b) { return static_cast<ZfocIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::Error>>(a) | static_cast<std::underlying_type_t<ZfocIntf::Error>>(b)); }
inline ZfocIntf::Error operator & (ZfocIntf::Error a, ZfocIntf::Error b) { return static_cast<ZfocIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::Error>>(a) & static_cast<std::underlying_type_t<ZfocIntf::Error>>(b)); }
inline ZfocIntf::Error operator ^ (ZfocIntf::Error a, ZfocIntf::Error b) { return static_cast<ZfocIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::Error>>(a) ^ static_cast<std::underlying_type_t<ZfocIntf::Error>>(b)); }
inline ZfocIntf::Error& operator |= (ZfocIntf::Error &a, ZfocIntf::Error b) { return reinterpret_cast<ZfocIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::Error>&>(a) |= static_cast<std::underlying_type_t<ZfocIntf::Error>>(b)); }
inline ZfocIntf::Error& operator &= (ZfocIntf::Error &a, ZfocIntf::Error b) { return reinterpret_cast<ZfocIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::Error>&>(a) &= static_cast<std::underlying_type_t<ZfocIntf::Error>>(b)); }
inline ZfocIntf::Error& operator ^= (ZfocIntf::Error &a, ZfocIntf::Error b) { return reinterpret_cast<ZfocIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::Error>&>(a) ^= static_cast<std::underlying_type_t<ZfocIntf::Error>>(b)); }
inline ZfocIntf::Error operator ~ (ZfocIntf::Error a) { return static_cast<ZfocIntf::Error>(~static_cast<std::underlying_type_t<ZfocIntf::Error>>(a)); }
// this is technically not thread-safe but practically it might be
inline ZfocIntf::CanIntf::Error operator | (ZfocIntf::CanIntf::Error a, ZfocIntf::CanIntf::Error b) { return static_cast<ZfocIntf::CanIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(a) | static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(b)); }
inline ZfocIntf::CanIntf::Error operator & (ZfocIntf::CanIntf::Error a, ZfocIntf::CanIntf::Error b) { return static_cast<ZfocIntf::CanIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(a) & static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(b)); }
inline ZfocIntf::CanIntf::Error operator ^ (ZfocIntf::CanIntf::Error a, ZfocIntf::CanIntf::Error b) { return static_cast<ZfocIntf::CanIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(a) ^ static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(b)); }
inline ZfocIntf::CanIntf::Error& operator |= (ZfocIntf::CanIntf::Error &a, ZfocIntf::CanIntf::Error b) { return reinterpret_cast<ZfocIntf::CanIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>&>(a) |= static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(b)); }
inline ZfocIntf::CanIntf::Error& operator &= (ZfocIntf::CanIntf::Error &a, ZfocIntf::CanIntf::Error b) { return reinterpret_cast<ZfocIntf::CanIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>&>(a) &= static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(b)); }
inline ZfocIntf::CanIntf::Error& operator ^= (ZfocIntf::CanIntf::Error &a, ZfocIntf::CanIntf::Error b) { return reinterpret_cast<ZfocIntf::CanIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>&>(a) ^= static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(b)); }
inline ZfocIntf::CanIntf::Error operator ~ (ZfocIntf::CanIntf::Error a) { return static_cast<ZfocIntf::CanIntf::Error>(~static_cast<std::underlying_type_t<ZfocIntf::CanIntf::Error>>(a)); }
// this is technically not thread-safe but practically it might be
inline ZfocIntf::AxisIntf::Error operator | (ZfocIntf::AxisIntf::Error a, ZfocIntf::AxisIntf::Error b) { return static_cast<ZfocIntf::AxisIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(a) | static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(b)); }
inline ZfocIntf::AxisIntf::Error operator & (ZfocIntf::AxisIntf::Error a, ZfocIntf::AxisIntf::Error b) { return static_cast<ZfocIntf::AxisIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(a) & static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(b)); }
inline ZfocIntf::AxisIntf::Error operator ^ (ZfocIntf::AxisIntf::Error a, ZfocIntf::AxisIntf::Error b) { return static_cast<ZfocIntf::AxisIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(a) ^ static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(b)); }
inline ZfocIntf::AxisIntf::Error& operator |= (ZfocIntf::AxisIntf::Error &a, ZfocIntf::AxisIntf::Error b) { return reinterpret_cast<ZfocIntf::AxisIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>&>(a) |= static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(b)); }
inline ZfocIntf::AxisIntf::Error& operator &= (ZfocIntf::AxisIntf::Error &a, ZfocIntf::AxisIntf::Error b) { return reinterpret_cast<ZfocIntf::AxisIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>&>(a) &= static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(b)); }
inline ZfocIntf::AxisIntf::Error& operator ^= (ZfocIntf::AxisIntf::Error &a, ZfocIntf::AxisIntf::Error b) { return reinterpret_cast<ZfocIntf::AxisIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>&>(a) ^= static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(b)); }
inline ZfocIntf::AxisIntf::Error operator ~ (ZfocIntf::AxisIntf::Error a) { return static_cast<ZfocIntf::AxisIntf::Error>(~static_cast<std::underlying_type_t<ZfocIntf::AxisIntf::Error>>(a)); }
// this is technically not thread-safe but practically it might be
inline ZfocIntf::MotorIntf::Error operator | (ZfocIntf::MotorIntf::Error a, ZfocIntf::MotorIntf::Error b) { return static_cast<ZfocIntf::MotorIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(a) | static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(b)); }
inline ZfocIntf::MotorIntf::Error operator & (ZfocIntf::MotorIntf::Error a, ZfocIntf::MotorIntf::Error b) { return static_cast<ZfocIntf::MotorIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(a) & static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(b)); }
inline ZfocIntf::MotorIntf::Error operator ^ (ZfocIntf::MotorIntf::Error a, ZfocIntf::MotorIntf::Error b) { return static_cast<ZfocIntf::MotorIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(a) ^ static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(b)); }
inline ZfocIntf::MotorIntf::Error& operator |= (ZfocIntf::MotorIntf::Error &a, ZfocIntf::MotorIntf::Error b) { return reinterpret_cast<ZfocIntf::MotorIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>&>(a) |= static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(b)); }
inline ZfocIntf::MotorIntf::Error& operator &= (ZfocIntf::MotorIntf::Error &a, ZfocIntf::MotorIntf::Error b) { return reinterpret_cast<ZfocIntf::MotorIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>&>(a) &= static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(b)); }
inline ZfocIntf::MotorIntf::Error& operator ^= (ZfocIntf::MotorIntf::Error &a, ZfocIntf::MotorIntf::Error b) { return reinterpret_cast<ZfocIntf::MotorIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>&>(a) ^= static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(b)); }
inline ZfocIntf::MotorIntf::Error operator ~ (ZfocIntf::MotorIntf::Error a) { return static_cast<ZfocIntf::MotorIntf::Error>(~static_cast<std::underlying_type_t<ZfocIntf::MotorIntf::Error>>(a)); }
// this is technically not thread-safe but practically it might be
inline ZfocIntf::ControllerIntf::Error operator | (ZfocIntf::ControllerIntf::Error a, ZfocIntf::ControllerIntf::Error b) { return static_cast<ZfocIntf::ControllerIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(a) | static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(b)); }
inline ZfocIntf::ControllerIntf::Error operator & (ZfocIntf::ControllerIntf::Error a, ZfocIntf::ControllerIntf::Error b) { return static_cast<ZfocIntf::ControllerIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(a) & static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(b)); }
inline ZfocIntf::ControllerIntf::Error operator ^ (ZfocIntf::ControllerIntf::Error a, ZfocIntf::ControllerIntf::Error b) { return static_cast<ZfocIntf::ControllerIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(a) ^ static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(b)); }
inline ZfocIntf::ControllerIntf::Error& operator |= (ZfocIntf::ControllerIntf::Error &a, ZfocIntf::ControllerIntf::Error b) { return reinterpret_cast<ZfocIntf::ControllerIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>&>(a) |= static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(b)); }
inline ZfocIntf::ControllerIntf::Error& operator &= (ZfocIntf::ControllerIntf::Error &a, ZfocIntf::ControllerIntf::Error b) { return reinterpret_cast<ZfocIntf::ControllerIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>&>(a) &= static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(b)); }
inline ZfocIntf::ControllerIntf::Error& operator ^= (ZfocIntf::ControllerIntf::Error &a, ZfocIntf::ControllerIntf::Error b) { return reinterpret_cast<ZfocIntf::ControllerIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>&>(a) ^= static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(b)); }
inline ZfocIntf::ControllerIntf::Error operator ~ (ZfocIntf::ControllerIntf::Error a) { return static_cast<ZfocIntf::ControllerIntf::Error>(~static_cast<std::underlying_type_t<ZfocIntf::ControllerIntf::Error>>(a)); }
// this is technically not thread-safe but practically it might be
inline ZfocIntf::EncoderIntf::Error operator | (ZfocIntf::EncoderIntf::Error a, ZfocIntf::EncoderIntf::Error b) { return static_cast<ZfocIntf::EncoderIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(a) | static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(b)); }
inline ZfocIntf::EncoderIntf::Error operator & (ZfocIntf::EncoderIntf::Error a, ZfocIntf::EncoderIntf::Error b) { return static_cast<ZfocIntf::EncoderIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(a) & static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(b)); }
inline ZfocIntf::EncoderIntf::Error operator ^ (ZfocIntf::EncoderIntf::Error a, ZfocIntf::EncoderIntf::Error b) { return static_cast<ZfocIntf::EncoderIntf::Error>(static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(a) ^ static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(b)); }
inline ZfocIntf::EncoderIntf::Error& operator |= (ZfocIntf::EncoderIntf::Error &a, ZfocIntf::EncoderIntf::Error b) { return reinterpret_cast<ZfocIntf::EncoderIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>&>(a) |= static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(b)); }
inline ZfocIntf::EncoderIntf::Error& operator &= (ZfocIntf::EncoderIntf::Error &a, ZfocIntf::EncoderIntf::Error b) { return reinterpret_cast<ZfocIntf::EncoderIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>&>(a) &= static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(b)); }
inline ZfocIntf::EncoderIntf::Error& operator ^= (ZfocIntf::EncoderIntf::Error &a, ZfocIntf::EncoderIntf::Error b) { return reinterpret_cast<ZfocIntf::EncoderIntf::Error&>(reinterpret_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>&>(a) ^= static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(b)); }
inline ZfocIntf::EncoderIntf::Error operator ~ (ZfocIntf::EncoderIntf::Error a) { return static_cast<ZfocIntf::EncoderIntf::Error>(~static_cast<std::underlying_type_t<ZfocIntf::EncoderIntf::Error>>(a)); }

#pragma GCC pop_options

#endif // __INTERFACES_HPP
