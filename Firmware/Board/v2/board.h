/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_H
#define __BOARD_H

#include <stdbool.h>

// STM specific includes
#include <stm32g4xx_hal.h>
#include <gpio.h>
#include <tim.h>
#include <fdcan.h>
#include <main.h>
#include "cmsis_os.h"

#include <stm32_system.h>

#define SHUNT_RESISTANCE (8e-3f)    // [Ohm] shunt resistor value

#define AXIS_COUNT (2)

#define GPIO_COUNT (10)

#define DEFAULT_BRAKE_RESISTANCE (10.0f) // [Ohm] physical resistor on the board

#define DEFAULT_MIN_DC_VOLTAGE 0.0f

#define DEFAULT_GPIO_MODES \
    ZfocIntf::GPIO_MODE_DIGITAL,          /* GPIO0  */ \
    ZfocIntf::GPIO_MODE_UART,             /* GPIO1  */ \
    ZfocIntf::GPIO_MODE_UART,             /* GPIO2  */ \
    ZfocIntf::GPIO_MODE_ENC,              /* GPIO3  */ \
    ZfocIntf::GPIO_MODE_CAN_B,              /* GPIO4  */ \
    ZfocIntf::GPIO_MODE_CAN_B,              /* GPIO5  */ \
    ZfocIntf::GPIO_MODE_CAN_A,              /* GPIO6  */ \
    ZfocIntf::GPIO_MODE_CAN_A,              /* GPIO7  */ \
    ZfocIntf::GPIO_MODE_ENC,              /* GPIO8  */ \
    ZfocIntf::GPIO_MODE_DIGITAL,          /* GPIO9  */ 

#define TIM_TIME_BASE TIM17

// Run control loop at the same frequency as the current measurement
#define CONTROL_TIMER_PERIOD_TICKS (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1))

#define TIM1_INIT_COUNT (TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128) // TODO: explain why this offset

// Maximum allowed delay between control loop update and current update before
#define MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA (TIM_1_8_PERIOD_CLOCKS / 2 + 1 * 128)

#ifdef __cplusplus
#include <stm32_gpio.hpp>
#include <task_timer.hpp>

#include <motor.hpp>
#include <encoder.hpp>

extern std::array<Axis, AXIS_COUNT> axes;
extern Motor motors[AXIS_COUNT];
extern Encoder encoders[AXIS_COUNT];
extern Stm32Gpio gpios[GPIO_COUNT];

struct GpioFunction { int mode = 0; uint8_t alternate_function = 0xff; };
extern std::array<GpioFunction, 3> alternate_functions[GPIO_COUNT];

#endif

// Period in [s]
#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )
static const int current_meas_hz = (int)CURRENT_MEAS_HZ;

#define VBUS_S_DIVIDER_RATIO 11.0f

// Linear range of the DRV8301 opamp output: 0.3V...5.7V. We set the upper limit
// to 3.0V so that it's symmetric around the center point of 1.65V.
#define CURRENT_SENSE_MIN_VOLT  0.0f
#define CURRENT_SENSE_MAX_VOLT  3.3f

// This board has no board-specific user configurations
static inline bool board_read_config() { return true; }
static inline bool board_write_config() { return true; }
static inline void board_clear_config() { }
static inline bool board_apply_config() { return true; }

void system_init();
bool board_init();
void start_timers();

#endif // __BOARD_H

