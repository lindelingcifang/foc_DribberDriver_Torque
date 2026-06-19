
#include "zfoc_main.h"
#include <algorithm>
#include <numeric>

float closed_torque_debug = 0.0f; // debug
float vel_int_torq_debug = 0.0f; // debug
float vel_err_debug = 0.0f; // debug
float pos_err_debug = 0.0f; // debug

bool Controller::apply_config() {
    config_.parent = this;
    update_filter_gains();
    return true;
}

void Controller::reset() {
    // pos_setpoint is initialized in start_closed_loop_control
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
    mechanical_power_ = 0.0f;
    electrical_power_ = 0.0f;
}

void Controller::set_error(Error error) {
    error_ |= error;
    last_error_time_ = zfoc.n_evt_control_loop_ * current_meas_period;
}

//--------------------------------
// Command Handling
//--------------------------------


void Controller::move_to_pos(float goal_point) {
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_traj_.config_.vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
}

void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE) {
        config_.anticogging.calib_anticogging = true;
    }
}

float Controller::remove_anticogging_bias()
{
    auto& cogmap = config_.anticogging.cogging_map;
    
    auto sum = std::accumulate(std::begin(cogmap), std::end(cogmap), 0.0f);
    auto average = sum / std::size(cogmap);

    for(auto& val : cogmap) {
        val -= average;
    }

    return average;
}


/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    float pos_err = input_pos_ - pos_estimate;
    if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / (float)axis_->encoder_.config_.cpr &&
        std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / (float)axis_->encoder_.config_.cpr) {
        config_.anticogging.cogging_map[std::clamp<uint32_t>(config_.anticogging.index++, 0, 300)] = vel_integrator_torque_;
    }
    if (config_.anticogging.index < 300) {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * axis_->encoder_.getCoggingRatio();
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    } else {
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = 0.0f;  // Send the motor home
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        anticogging_valid_ = true;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

void Controller::set_input_pos_and_steps(float const pos) {
    input_pos_ = pos;
    if (config_.circular_setpoints) {
        float const range = config_.circular_setpoint_range;
        axis_->steps_ = (int64_t)(fmodf_pos(pos, range) / range * config_.steps_per_circular_range);
    } else {
        axis_->steps_ = (int64_t)(pos * config_.steps_per_circular_range);
    }
}

bool Controller::control_mode_updated() {
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        std::optional<float> estimate = (config_.circular_setpoints ?
                                pos_estimate_circular_src_ :
                                pos_estimate_linear_src_).any();
        if (!estimate.has_value()) {
            return false;
        }

        pos_setpoint_ = *estimate;
        set_input_pos_and_steps(*estimate);
    }
    return true;
}

void Controller::update_filter_gains() {
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

// Asymmetric velocity limit for dribbler.
// Overspeed branch only: when vel < v_min, add counter-torque (bump-less).
// No action on slow speed (vel > v_max is handled by the torque command itself).
static float limitVelAsymmetric(const float v_min, const float v_max,
                                 const float vel_estimate, const float vel_gain,
                                 const float torque) {
    if (vel_estimate < v_min) {
        float overspeed = v_min - vel_estimate;
        float counter_torque = overspeed * vel_gain;
        float modified_torque = torque + counter_torque;

        // Anti-spit clamp: never output positive torque (would brake/reverse the dribbler)
        // if (modified_torque > 0.0f) {
        //     return 0.0f;
        // }
        return modified_torque;
    }
    return torque;
}

float Controller::calculateDynamicVMin(float v_chassis_x) {
    constexpr float BASE_V_MIN = -50.0f;
    constexpr float PER_MS_COMPENSATE = 1.0f / 0.04335f; // ~23.07 turn/s per m/s
    constexpr float SLIP_MARGIN = 1.3f;
    constexpr float DEAD_ZONE = 0;       // m/s, backward=negative
    constexpr float FILTER_ALPHA = 0.02f;
    constexpr float SAFETY_CLAMP = -200.0f;   // turn/s

    // LP filter chassis speed (1st order IIR)
    chassis_speed_filtered_ += FILTER_ALPHA * (v_chassis_x - chassis_speed_filtered_);

    float dynamic_v_min = BASE_V_MIN;
    if (chassis_speed_filtered_ < DEAD_ZONE) {
        float compensate_turns = -chassis_speed_filtered_ * PER_MS_COMPENSATE * SLIP_MARGIN;
        dynamic_v_min -= compensate_turns;
    }
    if (dynamic_v_min < SAFETY_CLAMP) {
        dynamic_v_min = SAFETY_CLAMP;
    }
    return dynamic_v_min;
}

float Controller::applyTorqueSlewRate(float target, float dt) {
    constexpr float SLEW_RELEASE = 50.0f;  // [Nm/s] fast release toward 0
    constexpr float SLEW_APPLY  = 5.0f;    // [Nm/s] slow apply toward negative

    float step = target - torque_slew_current_;
    if (step > 0.0f) {
        // releasing (toward 0): fast
        step = std::min(step, SLEW_RELEASE * dt);
    } else {
        // applying (more negative): slow
        step = std::max(step, -SLEW_APPLY * dt);
    }
    torque_slew_current_ += step;
    return torque_slew_current_;
}

bool Controller::update() {
    if (axis_->encoder_.config_.mode == Encoder::MODE_DISABLED
        && !axis_->config_.enable_sensorless_mode) {
        torque_output_ = 0.0f;
        error_ &= ~ERROR_INVALID_ESTIMATE;
        return true;
    }

    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.present();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.present();
    std::optional<float> pos_wrap = pos_wrap_src_.present();
    std::optional<float> vel_estimate = vel_estimate_src_.present();

    std::optional<float> anticogging_pos_estimate = axis_->encoder_.pos_estimate_.present();
    std::optional<float> anticogging_vel_estimate = axis_->encoder_.vel_estimate_.present();

    if (axis_->step_dir_active_) {
        if (config_.circular_setpoints) {
            if (!pos_wrap.has_value()) {
                set_error(ERROR_INVALID_CIRCULAR_RANGE);
                return false;
            }
            input_pos_ = (float)(axis_->steps_ % config_.steps_per_circular_range) * (*pos_wrap / (float)(config_.steps_per_circular_range));
        } else {
            input_pos_ = (float)(axis_->steps_) / (float)(config_.steps_per_circular_range);
        }
    }

    if (config_.anticogging.calib_anticogging) {
        if (!anticogging_pos_estimate.has_value() || !anticogging_vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints) {
        if (!pos_wrap.has_value()) {
            set_error(ERROR_INVALID_CIRCULAR_RANGE);
            return false;
        }
        input_pos_ = fmodf_pos(input_pos_, *pos_wrap);
    }

    // Update inputs
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            if (config_.circular_setpoints) {
                if (!pos_wrap.has_value()) {
                    set_error(ERROR_INVALID_CIRCULAR_RANGE);
                    return false;
                }
                delta_pos = wrap_pm(delta_pos, *pos_wrap);
            }
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror < AXIS_COUNT) {
                std::optional<float> other_pos = axes[config_.axis_to_mirror].encoder_.pos_estimate_.present();
                std::optional<float> other_vel = axes[config_.axis_to_mirror].encoder_.vel_estimate_.present();
                std::optional<float> other_torque = axes[config_.axis_to_mirror].controller_.torque_output_.present();

                if (!other_pos.has_value() || !other_vel.has_value() || !other_torque.has_value()) {
                    set_error(ERROR_INVALID_ESTIMATE);
                    return false;
                }

                pos_setpoint_ = *other_pos * config_.mirror_ratio;
                vel_setpoint_ = *other_vel * config_.mirror_ratio;
                torque_setpoint_ = *other_torque * config_.torque_mirror_ratio;
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = axis_->trap_traj_.Xf_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos_estimate = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        case INPUT_MODE_TUNING: {
            autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * current_meas_period));
            float c = 0;
            float s = 0;
            cordic_cos_sin(autotuning_phase_, &c, &s);
            pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
            vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
            torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Never command a setpoint beyond its limit
    if(config_.enable_vel_limit) {
        vel_setpoint_ = std::clamp(vel_setpoint_, -config_.vel_limit, config_.vel_limit);
    }
    const float Tlim = axis_->motor_.max_available_torque();
    torque_setpoint_ = std::clamp(torque_setpoint_, -Tlim, Tlim);

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        float pos_err;

        if (config_.circular_setpoints) {
            if (!pos_estimate_circular.has_value() || !pos_wrap.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);
            // Circular delta
            pos_err = *pos_estimate_circular - pos_setpoint_;
            pos_err = wrap_pm(pos_err, *pos_wrap);
        } else {
            if (!pos_estimate_linear.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
        }

        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        if (!anticogging_pos_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        float anticogging_pos = *anticogging_pos_estimate / axis_->encoder_.getCoggingRatio();
        torque += config_.anticogging.cogging_map[std::clamp(mod((int)anticogging_pos, 300), 0, 300)];
    }

    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;

        if (axis_->axis_num_ == 5) {
            //debug
            vel_err_debug = v_err;
        }
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_torque_mode_vel_limit) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (config_.enable_dribbler_vel_limit) {
            // Dribbler asymmetric velocity limit: v_min=dynamic (chassis-speed compensated)
            float dynamic_v_min = calculateDynamicVMin(chassis_speed_);
            torque = limitVelAsymmetric(dynamic_v_min, config_.dribbler_vel_limit_upper,
                                        *vel_estimate, vel_gain, torque);
        } else {
            torque = limitVel(config_.vel_limit, *vel_estimate, vel_gain, torque);
        }
    }

    // Torque limiting
    bool limited = false;
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }

    // Asymmetric torque limit (e.g. dribbler ball suction: [-0.05, 0])
    if (config_.torque_limit_min > -INFINITY || config_.torque_limit_max < INFINITY) {
        float clamped = std::clamp(torque, config_.torque_limit_min, config_.torque_limit_max);
        if (clamped != torque) {
            limited = true;
            torque = clamped;
        }
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
        // integrator limiting to prevent windup 
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }
    if (axis_->axis_num_ == 0) {
        //debug
        vel_int_torq_debug = vel_integrator_torque_;
    }

    float ideal_electrical_power = 0.0f;
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) {
        ideal_electrical_power = axis_->motor_.current_control_.power_ - \
            SQ(axis_->motor_.current_control_.Iq_measured_) * 1.5f * axis_->motor_.config_.phase_resistance - \
            SQ(axis_->motor_.current_control_.Id_measured_) * 1.5f * axis_->motor_.config_.phase_resistance;
    }
    else {
        ideal_electrical_power = axis_->motor_.current_control_.power_;
    }
    mechanical_power_ += config_.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    electrical_power_ += config_.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

    // Spinout check
    // If mechanical power is negative (braking) and measured power is positive, something is wrong
    // This indicates that the controller is trying to stop, but torque is being produced.
    // Usually caused by an incorrect encoder offset
    if (mechanical_power_ < config_.spinout_mechanical_power_threshold && electrical_power_ > config_.spinout_electrical_power_threshold) {
        set_error(ERROR_SPINOUT_DETECTED);
        return false;
    }

    // Slew rate limiting: fast release, slow apply (anti-oscillation)
    torque = applyTorqueSlewRate(torque, current_meas_period);

    torque_output_ = torque;
    if (axis_->axis_num_ == 5) {
        //debug
        closed_torque_debug = torque;
    }

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}
