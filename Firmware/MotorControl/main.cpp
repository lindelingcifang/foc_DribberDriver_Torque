
#define __MAIN_CPP__
#include "zfoc_main.h"
#include "nvm_config.hpp"

#include "freertos_vars.h"
#include "interface_can.hpp"

osSemaphoreId sem_can;
const uint32_t stack_size_default_task = 512;

#if defined(STM32G474xx)
// Place FreeRTOS heap in core coupled memory for better performance
__attribute__((section(".ccmram")))
#endif
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

uint32_t _reboot_cookie __attribute__ ((section (".noinit")));
extern char _estack; // provided by the linker script

Zfoc zfoc{};

ConfigManager config_manager;

static bool config_read_all() {
    bool success = board_read_config() &&
           config_manager.read(&zfoc.config_) &&
           config_manager.read(&zfoc.can_a.config_) &&
           config_manager.read(&zfoc.can_b.config_);
    for (size_t i = 0; i < AXIS_COUNT; i++) {
        success = success && config_manager.read(&encoders[i].config_) &&
                  config_manager.read(&axes[i].controller_.config_) &&
                  config_manager.read(&axes[i].trap_traj_.config_) &&
                  config_manager.read(&motors[i].config_) &&
                  config_manager.read(&axes[i].config_);
    }
    return success;
}

static bool config_write_all() {
    bool success = board_write_config() &&
           config_manager.write(&zfoc.config_) &&
           config_manager.write(&zfoc.can_a.config_) &&
           config_manager.write(&zfoc.can_b.config_);
    for (size_t i = 0; i < AXIS_COUNT; i++) {
        success = config_manager.write(&encoders[i].config_) &&
                  config_manager.write(&axes[i].controller_.config_) &&
                  config_manager.write(&axes[i].trap_traj_.config_) &&
                  config_manager.write(&motors[i].config_) &&
                  config_manager.write(&axes[i].config_);
    }
    return success;
}

static void config_clear_all() {
    zfoc.config_ = {};
    zfoc.can_a.config_ = {};
    zfoc.can_b.config_ = {};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        // encoders[i].config_ = {};
        axes[i].controller_.config_ = {};
        axes[i].controller_.config_.load_encoder_axis = i;
        axes[i].trap_traj_.config_ = {};
        motors[i].config_ = {};
        axes[i].clear_config();
    }
    // Axis 0 (dribbler): torque control with asymmetric velocity limit [-50, vel_lower from CAN]
    axes[0].controller_.config_.control_mode = Controller::CONTROL_MODE_TORQUE_CONTROL;
    axes[0].controller_.config_.input_mode = Controller::INPUT_MODE_PASSTHROUGH;
    axes[0].controller_.config_.torque_limit_min = -0.25f;
    axes[0].controller_.config_.torque_limit_max = 0.25f;
    axes[0].controller_.config_.enable_dribbler_vel_limit = true;
}

static bool config_apply_all() {
    bool success = zfoc.can_a.apply_config();
    success = success && zfoc.can_b.apply_config();
    for (size_t i = 0; (i < AXIS_COUNT) && success; ++i) {
        success = encoders[i].apply_config(motors[i].config_.motor_type)
               && axes[i].controller_.apply_config()
               && motors[i].apply_config()
               && axes[i].apply_config();
    }
    return success;
}

bool Zfoc::save_configuration(void) {
    bool success;

    CRITICAL_SECTION() {
        bool any_armed = std::any_of(axes.begin(), axes.end(),
            [](auto& axis){ return axis.motor_.is_armed_; });
        if (any_armed) {
            return false;
        }

        size_t config_size = 0;
        success = config_manager.prepare_store()
               && config_write_all()
               && config_manager.start_store(&config_size)
               && config_write_all()
               && config_manager.finish_store();

        // FIXME: during save_configuration we might miss some interrupts
        // because the CPU gets halted during a flash erase. Missing events
        // (encoder updates, step/dir steps) is not good so to be sure we just
        // reboot.
        NVIC_SystemReset();
    }

    return success;
}

void Zfoc::erase_configuration(void) {
    NVM_erase();

    // FIXME: this reboot is a workaround because we don't want the next save_configuration
    // to write back the old configuration from RAM to NVM. The proper action would
    // be to reset the values in RAM to default. However right now that's not
    // practical because several startup actions depend on the config. The
    // other problem is that the stack overflows if we reset to default here.
    NVIC_SystemReset();
}

bool Zfoc::any_error() {
    return error_ != Zfoc::ERROR_NONE
        || std::any_of(axes.begin(), axes.end(), [](Axis& axis){
            return axis.error_ != Axis::ERROR_NONE
                || axis.motor_.error_ != Motor::ERROR_NONE
                || axis.encoder_.error_ != Encoder::ERROR_NONE
                || axis.controller_.error_ != Controller::ERROR_NONE;
        });
}

void Zfoc::clear_errors() {
    error_ = Zfoc::ERROR_NONE;
    for (auto& axis : axes) {
        axis.error_ = Axis::ERROR_NONE;
        axis.motor_.error_ = Motor::ERROR_NONE;
        axis.encoder_.error_ = Encoder::ERROR_NONE;
        axis.encoder_.spi_error_rate_ = 0.0f;
        axis.controller_.error_ = Controller::ERROR_NONE;
    }
    if (zfoc.config_.enable_brake_resistor) {
        safety_critical_arm_brake_resistor();
    }
}

extern "C" {

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) {

    for(auto& axis: axes){
        axis.motor_.disarm();
    }
    safety_critical_disarm_brake_resistor();
    for (;;); // TODO: safe action
}

void vApplicationIdleHook(void) {
    volatile uint32_t free_heap = xPortGetFreeHeapSize();
    if (zfoc.system_stats_.fully_booted) {
        zfoc.system_stats_.uptime = xTaskGetTickCount();
        zfoc.system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();

        uint32_t min_stack_space[AXIS_COUNT];
        std::transform(axes.begin(), axes.end(), std::begin(min_stack_space), [](auto& axis) { return uxTaskGetStackHighWaterMark(axis.thread_id_) * sizeof(StackType_t); });
        zfoc.system_stats_.max_stack_usage_axis = axes[0].stack_size_ - *std::min_element(std::begin(min_stack_space), std::end(min_stack_space));
        zfoc.system_stats_.max_stack_usage_startup = stack_size_default_task - uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
        zfoc.system_stats_.max_stack_usage_can_a = zfoc.can_a.stack_size_ - uxTaskGetStackHighWaterMark(zfoc.can_a.thread_id_) * sizeof(StackType_t);
        zfoc.system_stats_.max_stack_usage_can_b = zfoc.can_b.stack_size_ - uxTaskGetStackHighWaterMark(zfoc.can_b.thread_id_) * sizeof(StackType_t);

        zfoc.system_stats_.stack_size_axis = axes[0].stack_size_;
        zfoc.system_stats_.stack_size_startup = stack_size_default_task;
        zfoc.system_stats_.stack_size_can_a = zfoc.can_a.stack_size_;
        zfoc.system_stats_.stack_size_can_b = zfoc.can_b.stack_size_;

        zfoc.system_stats_.prio_axis = osThreadGetPriority(axes[0].thread_id_);
        zfoc.system_stats_.prio_startup = osThreadGetPriority(defaultTaskHandle);
        zfoc.system_stats_.prio_can_a = osThreadGetPriority(zfoc.can_a.thread_id_);
        zfoc.system_stats_.prio_can_b = osThreadGetPriority(zfoc.can_b.thread_id_);
    }
}

} // extern "C"

/**
 * @brief [DEPRICATED] (get vbus from CANB) Runs system-level checks that need to be as real-time as possible.
 * 
 * This function is called after every current measurement of every motor.
 * It should finish as quickly as possible.
 */
void Zfoc::do_fast_checks() {
    if (!(vbus_voltage >= config_.dc_bus_undervoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_UNDER_VOLTAGE);
    if (!(vbus_voltage <= config_.dc_bus_overvoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_OVER_VOLTAGE);
}

/**
 * @brief Floats all power phases on the system (all motors and brake resistors).
 *
 * This should be called if a system level exception ocurred that makes it
 * unsafe to run power through the system in general.
 * 
 * TODO: send ERROR through CANB
 */
void Zfoc::disarm_with_error(Error error) {
    CRITICAL_SECTION() {
        for (auto& axis: axes) {
            axis.motor_.disarm_with_error(Motor::ERROR_SYSTEM_LEVEL);
        }
        safety_critical_disarm_brake_resistor();
        error_ |= error;
    }
}

/**
 * @brief Runs the periodic sampling tasks
 * 
 * All components that need to sample real-world data should do it in this
 * function as it runs on a high interrupt priority and provides lowest possible
 * timing jitter.
 * 
 * All function called from this function should adhere to the following rules:
 *  - Try to use the same number of CPU cycles in every iteration.
 *    (reason: Tasks that run later in the function still want lowest possible timing jitter)
 *  - Use as few cycles as possible.
 *    (reason: The interrupt blocks other important interrupts (TODO: which ones?))
 *  - Not call any FreeRTOS functions.
 *    (reason: The interrupt priority is higher than the max allowed priority for syscalls)
 * 
 * Time consuming and undeterministic logic/arithmetic should live on
 * control_loop_cb() instead.
 */
void Zfoc::sampling_cb() {
    n_evt_sampling_++;

    MEASURE_TIME(task_times_.sampling) {
        for (auto& axis: axes) {
            axis.encoder_.sample_now();
        }
    }
}

/**
 * @brief Runs the periodic control loop.
 * 
 * This function is executed in a low priority interrupt context and is allowed
 * to call CMSIS functions.
 * 
 * Yet it runs at a higher priority than communication workloads.
 * 
 * @param update_cnt: The true count of update events (wrapping around at 16
 *        bits). This is used for timestamp calculation in the face of
 *        potentially missed timer update interrupts. Therefore this counter
 *        must not rely on any interrupts.
 */
void Zfoc::control_loop_cb(uint32_t timestamp) {
    last_update_timestamp_ = timestamp;
    n_evt_control_loop_++;

    // TODO: use a configurable component list for most of the following things

    MEASURE_TIME(task_times_.control_loop_misc) {
        // Reset all output ports so that we are certain about the freshness of
        // all values that we use.
        // If we forget to reset a value here the worst that can happen is that
        // this safety check doesn't work.
        // TODO: maybe we should add a check to output ports that prevents
        // double-setting the value.
        for (auto& axis: axes) {
            axis.controller_.torque_output_.reset();
            axis.encoder_.phase_.reset();
            axis.encoder_.phase_vel_.reset();
            axis.encoder_.pos_estimate_.reset();
            axis.encoder_.vel_estimate_.reset();
            axis.encoder_.pos_circular_.reset();
            axis.motor_.Vdq_setpoint_.reset();
            axis.motor_.Idq_setpoint_.reset();
            axis.open_loop_controller_.Idq_setpoint_.reset();
            axis.open_loop_controller_.Vdq_setpoint_.reset();
            axis.open_loop_controller_.phase_.reset();
            axis.open_loop_controller_.phase_vel_.reset();
            axis.open_loop_controller_.total_distance_.reset();
        }
    }

    MEASURE_TIME(task_times_.control_loop_checks) {
        for (auto& axis: axes) {
            // look for errors at axis level and also all subcomponents
            bool checks_ok = axis.do_checks(timestamp);

            // make sure the watchdog is being fed. 
            bool watchdog_ok = axis.watchdog_check();

            if (!checks_ok || !watchdog_ok) {
                axis.motor_.disarm();
            }
        }
    }

    for (auto& axis: axes) {
        MEASURE_TIME(axis.task_times_.encoder_update)
            axis.encoder_.update();
    }

    // Controller of either axis might use the encoder estimate of the other
    // axis so we process both encoders before we continue.

    for (auto& axis: axes) {
        MEASURE_TIME(axis.task_times_.controller_update) {
            if (!axis.controller_.update()) { // uses position and velocity from encoder
                axis.error_ |= Axis::ERROR_CONTROLLER_FAILED;
            }
        }

        MEASURE_TIME(axis.task_times_.open_loop_controller_update)
            axis.open_loop_controller_.update(timestamp);

        MEASURE_TIME(axis.task_times_.motor_update)
            axis.motor_.update(timestamp); // uses torque from controller and phase_vel from encoder

        MEASURE_TIME(axis.task_times_.current_controller_update)
            axis.motor_.current_control_.update(timestamp); // uses the output of controller_ or open_loop_contoller_ and encoder_
    }

    // Tell the axis threads that the control loop has finished
    for (auto& axis: axes) {
        if (axis.thread_id_) {
            osSignalSet(axis.thread_id_, 0x0001);
        }
    }

}


/** @brief For diagnostics only */
uint32_t Zfoc::get_interrupt_status(int32_t irqn) {
    if ((irqn < -14) || (irqn >= 240)) {
        return 0xffffffff;
    }

    uint8_t priority = (irqn < -12)
        ? 0 // hard fault and NMI always have maximum priority
        : NVIC_GetPriority((IRQn_Type)irqn);
    uint32_t counter = GET_IRQ_COUNTER((IRQn_Type)irqn);
    bool is_enabled = (irqn < 0)
        ? true // processor interrupt vectors are always enabled
        : NVIC->ISER[(((uint32_t)(int32_t)irqn) >> 5UL)] & (uint32_t)(1UL << (((uint32_t)(int32_t)irqn) & 0x1FUL));
    
    return priority | ((counter & 0x7ffffff) << 8) | (is_enabled ? 0x80000000 : 0);
}



/**
 * @brief Main thread started from main().
 */
static void rtos_main(const void*) {
    //osDelay(100);
    // Init communications (this requires the axis objects to be constructed)
    init_communication();

    // Try to initialized gate drivers for fault-free startup.
    // If this does not succeed, a fault will be raised and the idle loop will
    // periodically attempt to reinit the gate driver.
    for(auto& axis: axes){
        axis.motor_.setup();
    }

    for(auto& axis: axes){
        axis.encoder_.setup();
    }

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    osDelay(10);

    // Wait for up to 2s for motor to become ready to allow for error-free
    // startup. This delay gives the current sensor calibration time to
    // converge. If the DRV chip is unpowered, the motor will not become ready
    // but we still enter idle state.
    for (size_t i = 0; i < 2000; ++i) {
        bool motors_ready = std::all_of(axes.begin(), axes.end(), [](auto& axis) {
            return axis.motor_.current_meas_.has_value();
        });
        if (motors_ready) {
            break;
        }
        osDelay(1);
    }

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].start_thread();
    }

    zfoc.system_stats_.fully_booted = true;

    // Main thread finished starting everything and can delete itself now (yes this is legal).
    vTaskDelete(defaultTaskHandle);
}

/**
 * @brief Carries out early startup tasks that need to run before any static
 * initializers.
 * This function gets called from the startup assembly code.
 */
extern "C" void early_start_checks(void) {
    /* The bootloader might fail to properly clean up after itself,
    so if we're not sure that the system is in a clean state we
    just reset it again */
    if(_reboot_cookie != 42) {
        _reboot_cookie = 42;
        NVIC_SystemReset();
    }
}

/**
 * @brief Main entry point called from assembly startup code.
 */
extern "C" int main(void) {
    // Init low level system functions (clocks, flash interface)
    system_init();

    // Load configuration from NVM. This needs to happen after system_init()
    // since the flash interface must be initialized and before board_init()
    // since board initialization can depend on the config.
    // size_t config_size = 0;
    // bool success = config_manager.start_load()
    //         && config_read_all()
    //         && config_manager.finish_load(&config_size)
    //         && config_apply_all();
    // if (success) {
    //     zfoc.user_config_loaded_ = config_size;
    // } else {
    //     config_clear_all();
    //     config_apply_all();
    // }
    config_clear_all();
    config_apply_all();

    // Init board-specific peripherals
    if (!board_init()) {
        for (;;); // TODO: handle properly
    }

    osSemaphoreDef(sem_can);
    sem_can = osSemaphoreCreate(osSemaphore(sem_can), 1);
    osSemaphoreWait(sem_can, 0);

    // Create main thread
    osThreadDef(defaultTask, rtos_main, osPriorityNormal, 0, stack_size_default_task / sizeof(StackType_t));
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
    volatile int free_stack = xPortGetFreeHeapSize();

    // Start scheduler
    osKernelStart();
    
    for (;;);
}








