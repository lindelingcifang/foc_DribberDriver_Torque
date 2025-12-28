
#include "open_loop_controller.hpp"
#include <board.h>

float open_phase_debug = 0.0f;
float open_phase_vel_debug = 0.0f;
float open_Vd_debug = 0.0f;
float open_Id_debug = 0.0f;
float open_Vq_debug = 0.0f;
float open_Iq_debug = 0.0f;

void OpenLoopController::update(uint32_t timestamp) {
    auto [prev_Id, prev_Iq] = Idq_setpoint_.previous().value_or(float2D{0.0f, 0.0f});
    auto [prev_Vd, prev_Vq] = Vdq_setpoint_.previous().value_or(float2D{0.0f, 0.0f});
    float phase = phase_.previous().value_or(initial_phase_);
    float phase_vel = phase_vel_.previous().value_or(0.0f);

    (void)prev_Iq; // unused
    (void)prev_Vq; // unused

    float dt = (float)(timestamp - timestamp_) / (float)TIM_1_8_CLOCK_HZ;
    
    Idq_setpoint_ = {
        std::clamp(target_current_, prev_Id - max_current_ramp_ * dt, prev_Id + max_current_ramp_ * dt),
        0.0f
    };
    Vdq_setpoint_ = {
        std::clamp(target_voltage_, prev_Vd - max_voltage_ramp_ * dt, prev_Vd + max_voltage_ramp_ * dt),
        0.0f
    };
    
    phase_vel = std::clamp(target_vel_, phase_vel - max_phase_vel_ramp_ * dt, phase_vel + max_phase_vel_ramp_ * dt);
    phase_vel_ = phase_vel;
    phase_ = wrap_pm_pi(phase + phase_vel * dt);
    total_distance_ = total_distance_.previous().value_or(0.0f) + phase_vel * dt;
    timestamp_ = timestamp;
    if (axis_num_ == 0) {
        //debug
        open_phase_debug = phase_.present().value_or(0.0f);
        open_phase_vel_debug = phase_vel_.present().value_or(0.0f);
        open_Vd_debug = Vdq_setpoint_.present().value_or(float2D{0.0f, 0.0f}).first;
        open_Id_debug = Idq_setpoint_.present().value_or(float2D{0.0f, 0.0f}).first;
        open_Vq_debug = Vdq_setpoint_.present().value_or(float2D{0.0f, 0.0f}).second;
        open_Iq_debug = Idq_setpoint_.present().value_or(float2D{0.0f, 0.0f}).second;
    }
}
