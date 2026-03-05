
#include "zfoc_main.h"
#include <Drivers/STM32/stm32_system.h>
#include <bitset>

// debug variables
uint32_t pos_abs_debug = 0;
float pos_estimate_debug = 0;
float vel_estimate_debug = 0;
float pose_cirular_debug = 0;
uint16_t raw_val_debug = 0;
float spi_error_rate_debug = 0.0f;
float phase_debug = 0.0f;

static constexpr uint16_t AS5047P_REG_ANGLECOM = 0x3FFFU;
static constexpr uint16_t AS5047P_REG_ANGLEUNC = 0x3FFEU;
static constexpr uint16_t AS5047P_REG_ERRFL    = 0x0001U;
static constexpr uint16_t AS5047P_REG_PROG     = 0x0003U;
static constexpr uint16_t AS5047P_REG_DIAAGC   = 0x3FFCU;
static constexpr uint16_t AS5047P_CMD_READ_BIT = 0x4000U;
static constexpr uint16_t AS5047P_PARITY_BIT   = 0x8000U;
static constexpr uint16_t AS5047P_DATA_MASK    = 0x3FFFU;
static constexpr uint16_t AS5047P_EF_MASK      = 0x4000U;

Encoder::Encoder(Stm32SpiArbiter* spi_arbiter, Stm32Gpio abs_spi_cs_gpio, Mode mode) :
        spi_arbiter_(spi_arbiter), abs_spi_cs_gpio_(abs_spi_cs_gpio),
        mode_(mode)
{
    config_.mode = mode_;
}

bool Encoder::apply_config(ZfocIntf::MotorIntf::MotorType motor_type) {
    config_.parent = this;

    switch(mode_) {
        case MODE_SPI_ABS_MT6701:
            config_.cpr = (1 << 13);
            break;
        case MODE_SPI_ABS_AS5047P:
            config_.cpr = (1 << 14);
            break;
        default:
            set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
            return false;
    }

    update_pll_gains();

    return true;
}

void Encoder::setup() {
    // Set in initialization
    // mode_ = config_.mode;

    spi_task_.config = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_16BIT,
        .CLKPolarity = SPI_POLARITY_HIGH,
        .CLKPhase = SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10,
    };

    switch (mode_) {
        case MODE_SPI_ABS_MT6701:
            abs_spi_dma_tx_[0] = 0xFFFFU;
            abs_spi_is_discontinuous_ = false;
            break;
        case MODE_SPI_ABS_AS5047P:
            spi_task_.config.CLKPolarity = SPI_POLARITY_LOW;
            spi_task_.config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
            as5047p_read_angle_cmd(abs_spi_dma_tx_);
            abs_spi_is_discontinuous_ = true;
            break;
        default:
            break;
    }

    if(mode_ & MODE_FLAG_ABS){
        abs_spi_cs_pin_init();
        if (axis_->controller_.config_.anticogging.pre_calibrated) {
            axis_->controller_.anticogging_valid_ = true;
        }
    }
}

void Encoder::set_error(Error error) {
    vel_estimate_valid_ = false;
    pos_estimate_valid_ = false;
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

//--------------------
// Hardware Dependent
//--------------------

// two order PLL equations:
// \dot{\hat{\theta}} = \hat{\omega} + k_p ( \theta_{meas} - \hat{\theta} )
// \dot{\hat{\omega}} = k_i ( \theta_{meas} - \hat{\theta} )
void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

void Encoder::check_pre_calibrated() {
    // TODO: restoring config from python backup is fragile here (ACIM motor type must be set first)
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_ACIM) {
        if (!is_ready_)
            config_.pre_calibrated = false;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_counts_ = (float)count;
    tim_cnt_sample_ = count;

    cpu_exit_critical(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    if (update_offset) {
        config_.phase_offset += count - count_in_cpr_;
        config_.phase_offset = mod(config_.phase_offset, config_.cpr);
    }

    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_counts_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

bool Encoder::run_direction_find() {
    int32_t init_enc_val = shadow_count_;

    Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
    lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
    lockin_config.finish_on_distance = true;
    lockin_config.finish_on_vel = false;
    bool success = axis_->run_lockin_spin(lockin_config, false);

    if (success) {
        // Check response and direction
        if (shadow_count_ > init_enc_val + 8) {
            // motor same dir as encoder
            config_.direction = 1;
        } else if (shadow_count_ < init_enc_val - 8) {
            // motor opposite dir as encoder
            config_.direction = -1;
        } else {
            config_.direction = 0;
        }
    }

    return success;
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
bool Encoder::run_offset_calibration() {
    const float start_lock_duration = 1.0f;

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    CRITICAL_SECTION() {
        // Reset state variables
        axis_->open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.phase_ = 0.0f;
        axis_->open_loop_controller_.phase_vel_ = 0.0f;

        float max_current_ramp = axis_->motor_.config_.calibration_current / start_lock_duration * 2.0f;
        axis_->open_loop_controller_.max_current_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_voltage_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_phase_vel_ramp_ = INFINITY;
        axis_->open_loop_controller_.target_current_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? axis_->motor_.config_.calibration_current : 0.0f;
        axis_->open_loop_controller_.target_voltage_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? 0.0f : axis_->motor_.config_.calibration_current;
        axis_->open_loop_controller_.target_vel_ = 0.0f;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(0 - config_.calib_scan_distance / 2.0f);

        axis_->motor_.current_control_.enable_current_control_src_ = (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        axis_->motor_.current_control_.Idq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Idq_setpoint_);
        axis_->motor_.current_control_.Vdq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Vdq_setpoint_);
        
        axis_->motor_.current_control_.phase_src_.connect_to(&axis_->open_loop_controller_.phase_);

        axis_->motor_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->motor_.current_control_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
    }
    axis_->wait_for_control_iteration();

    axis_->motor_.arm(&axis_->motor_.current_control_);

    // go to start position of forward scan for start_lock_duration to get ready to scan
    for (size_t i = 0; i < (size_t)(start_lock_duration * 1000.0f); ++i) {
        if (!axis_->motor_.is_armed_) {
            return false; // TODO: return "disarmed" error code
        }
        if (axis_->requested_state_ != Axis::AXIS_STATE_UNDEFINED) {
            axis_->motor_.disarm();
            return false; // TODO: return "aborted" error code
        }
        osDelay(1);
    }


    int32_t init_enc_val = shadow_count_;
    uint32_t num_steps = 0;
    int64_t encvaluesum = 0;

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = config_.calib_scan_omega;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
    }

    // scan forward
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(-INFINITY) >= config_.calib_scan_distance;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }

    // Check response and direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_NO_RESPONSE);
        axis_->motor_.disarm();
        return false;
    }

    // Check CPR
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    calib_scan_response_ = std::abs(shadow_count_ - init_enc_val);
    if (std::abs(calib_scan_response_ - expected_encoder_delta) / expected_encoder_delta > config_.calib_range) {
        set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
        axis_->motor_.disarm();
        return false;
    }

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = -config_.calib_scan_omega;
    }

    // scan backwards
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(INFINITY) <= 0.0f;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }

    // Motor disarmed because of an error
    if (!axis_->motor_.is_armed_) {
        return false;
    }

    axis_->motor_.disarm();

    config_.phase_offset = encvaluesum / num_steps;
    int32_t residual = encvaluesum - ((int64_t)config_.phase_offset * (int64_t)num_steps);
    config_.phase_offset_float = (float)residual / (float)num_steps + 0.5f;  // add 0.5 to center-align state to phase

    is_ready_ = true;
    return true;
}

void Encoder::sample_now() {
    switch (mode_) {
        case MODE_SPI_ABS_MT6701:
        case MODE_SPI_ABS_AS5047P: {
            abs_spi_start_transaction();
        } break;
        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }
}

bool Encoder::even_parity16(uint16_t value) {
    bool parity = false;
    while (value != 0U) {
        parity = !parity;
        value &= (uint16_t)(value - 1U);
    }
    return parity;
}

void Encoder::as5047p_read_angle_cmd(uint16_t *data) {
    uint16_t cmd = (uint16_t)((AS5047P_REG_ANGLEUNC & AS5047P_DATA_MASK) | AS5047P_CMD_READ_BIT);
    // uint16_t cmd = (uint16_t)((AS5047P_REG_ERRFL & AS5047P_DATA_MASK) | AS5047P_CMD_READ_BIT);
    if (even_parity16(cmd)) {
        cmd |= AS5047P_PARITY_BIT;
    }
    data[0] = cmd;
}

bool Encoder::abs_spi_start_transaction() {
    if (mode_ & MODE_FLAG_ABS){
        if (Stm32SpiArbiter::acquire_task(&spi_task_)) {
            spi_task_.ncs_gpio = abs_spi_cs_gpio_;
            spi_task_.tx_buf = (uint8_t*)abs_spi_dma_tx_;
            spi_task_.rx_buf = (uint8_t*)abs_spi_dma_rx_;
            spi_task_.length = 1;
            spi_task_.on_complete = [](void* ctx, bool success) { ((Encoder*)ctx)->abs_spi_cb(success); };
            spi_task_.on_complete_ctx = this;
            spi_task_.next = nullptr;
            spi_task_.is_discontinuous = abs_spi_is_discontinuous_;
            spi_arbiter_->transfer_async(&spi_task_);
        } else {
            return false;
        }
    }
    return true;
}

void Encoder::abs_spi_cb(bool success) {
    uint16_t pos;

    if (!success) {
        goto done;
    }

    switch (mode_) {
        case MODE_SPI_ABS_MT6701: {
            uint16_t rawVal = abs_spi_dma_rx_[0];
            raw_val_debug = rawVal;
            // Check status bits
            if (rawVal & 0x02) { // overspeed bit
                goto done;
            }
            pos = (rawVal >> 2) & 0x1FFF; // 14 bit position data
        } break;

        case MODE_SPI_ABS_AS5047P: {
            uint16_t frame = abs_spi_dma_rx_[0];
            raw_val_debug = frame;

            if (even_parity16(frame)) {
                goto done;
            }

            if ((frame & AS5047P_EF_MASK) != 0U) {
                goto done;
            }

            pos = frame & AS5047P_DATA_MASK;
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           goto done;
        } break;
    }

    pos_abs_ = pos;
    abs_spi_pos_updated_ = true;
    if (config_.pre_calibrated) {
        is_ready_ = true;
    }

done:
    Stm32SpiArbiter::release_task(&spi_task_);
}

void Encoder::abs_spi_cs_pin_init(){
    // Decode and init cs pin
    abs_spi_cs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

    // Write pin high
    abs_spi_cs_gpio_.write(true);
}

bool Encoder::update() {
    // update internal encoder state.
    int32_t delta_enc = 0;
    int32_t pos_abs_latched = pos_abs_; //LATCH

    switch (mode_) {
        case MODE_SPI_ABS_MT6701:
        case MODE_SPI_ABS_AS5047P: {
            if (abs_spi_pos_updated_ == false) {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                // debug variable
                spi_error_rate_debug = spi_error_rate_;
                // debug
                if (spi_error_rate_debug > 0.95f) {
                    volatile int a = 1;
                }
                // FIXME: Temporally disable error due to low reliability of MT6701 SPI
                // if (spi_error_rate_ > 0.05f) {
                //     set_error(ERROR_ABS_SPI_COM_FAIL);
                //     return false;
                // }
            } else {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
                // debug variable
                spi_error_rate_debug = spi_error_rate_;
            }

            // debug
            if (spi_error_rate_debug > 0.95f) {
                volatile int a = 1;
            }

            abs_spi_pos_updated_ = false;
            delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
            delta_enc = mod(delta_enc, config_.cpr);
            if (delta_enc > config_.cpr/2) {
                delta_enc -= config_.cpr;
            }

        }break;

        default: {
            set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
            return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    if(mode_ & MODE_FLAG_ABS)
        count_in_cpr_ = pos_abs_latched;

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
    // Encoder model
    auto encoder_model = [this](float internal_pos)->int32_t {
        return (int32_t)std::floor(internal_pos);
    };
    // discrete phase detector
    float delta_pos_counts = (float)(shadow_count_ - encoder_model(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - encoder_model(pos_cpr_counts_));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    // pll feedback
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (std::abs(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller
    pos_estimate_ = pos_estimate_counts_ / (float)config_.cpr;
    vel_estimate_ = vel_estimate_counts_ / (float)config_.cpr;

    // Low-pass filter velocity estimate to reduce noise in control loop
    // First-order IIR filter: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
    float vel_filter_alpha = std::min(current_meas_period * config_.vel_filter_bandwidth * 2.0f * M_PI, 1.0f);
    float vel_filtered = vel_estimate_filtered_.any().value_or(0.0f);
    vel_filtered += vel_filter_alpha * (vel_estimate_.any().value_or(0.0f) - vel_filtered);
    vel_estimate_filtered_ = vel_filtered;

    // debug variables
    if (axis_->axis_num_ == 0) {
        pos_abs_debug = pos_abs_;
        pos_estimate_debug = pos_estimate_counts_ / (float)config_.cpr;
        vel_estimate_debug = vel_filtered; // Use filtered velocity for debug
    }
    
    // TODO: we should strictly require that this value is from the previous iteration
    // to avoid spinout scenarios. However that requires a proper way to reset
    // the encoder from error states.
    float pos_circular = pos_circular_.any().value_or(0.0f);
    pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    pos_circular = fmodf_pos(pos_circular, axis_->controller_.config_.circular_setpoint_range);
    pos_circular_ = pos_circular;
    if (axis_->axis_num_ == 0) {
        pose_cirular_debug = pos_circular;
    }

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.phase_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.phase_offset_float);
    
    if (is_ready_) {
        phase_ = wrap_pm_pi(ph) * config_.direction;
        phase_vel_ = (2*M_PI) * *vel_estimate_.present() * axis_->motor_.config_.pole_pairs * config_.direction;

        if (axis_->axis_num_ == 0) {
        //debug
        phase_debug = phase_.present().value_or(0.0f);
        }
    }

    return true;
}
