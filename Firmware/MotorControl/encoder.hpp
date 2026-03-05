#ifndef __ENCODER_HPP
#define __ENCODER_HPP

class Encoder;

#include <board.h> // needed for arm_math.h
#include "utils.hpp"
#include <interfaces.hpp>
#include "component.hpp"
#include <stm32_uart.hpp>
#include <stm32_spi_arbiter.hpp>

class Encoder : public ZfocIntf::EncoderIntf {
public:
    static constexpr uint32_t MODE_FLAG_ABS = 0x100;

    struct Config_t {
        Mode mode = MODE_SPI_ABS_AS5047P;
        float calib_range = 0.025f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 100.0f;  // PLL bandwidth [Hz], reduced from 1000 for noise reduction
        float vel_filter_bandwidth = 500.0f; // Additional velocity low-pass filter bandwidth [Hz]
        int32_t phase_offset = 27;        // Offset between encoder count and rotor electrical phase
        float phase_offset_float = 0.501499832f; // Sub-count phase alignment offset
        int32_t cpr = 6 * 8; // Counts per revolution
        bool pre_calibrated = true; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        int32_t direction = 1; // direction with respect to motor
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state

        // custom setters
        Encoder* parent = nullptr;
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
    };

    Encoder(Stm32SpiArbiter* spi_arbiter, Stm32Gpio abs_spi_cs_gpio, Mode mode);
    
    bool apply_config(ZfocIntf::MotorIntf::MotorType motor_type);
    void setup();
    void set_error(Error error);
    bool do_checks();

    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);

    bool run_direction_find();
    bool run_offset_calibration();
    void sample_now();
    bool update();

    Stm32SpiArbiter* spi_arbiter_;
    Stm32Gpio abs_spi_cs_gpio_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Config_t config_;

    Error error_ = ERROR_NONE;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    OutputPort<float> phase_ = 0.0f;     // [rad]
    OutputPort<float> phase_vel_ = 0.0f; // [rad/s]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float delta_pos_cpr_counts_ = 0.0f;  // [count] phase detector result for debug
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f; // fraction of failed SPI transactions

    OutputPort<float> pos_estimate_ = 0.0f; // [turn]
    OutputPort<float> vel_estimate_ = 0.0f; // [turn/s] raw PLL output
    OutputPort<float> vel_estimate_filtered_ = 0.0f; // [turn/s] low-pass filtered velocity
    OutputPort<float> pos_circular_ = 0.0f; // [turn]

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    int16_t tim_cnt_sample_ = 0; // 
    static const constexpr GPIO_TypeDef* ports_to_sample[] = { GPIOA, GPIOB, GPIOC };
    uint16_t port_samples_[sizeof(ports_to_sample) / sizeof(ports_to_sample[0])];

    bool abs_spi_start_transaction();
    void abs_spi_cb(bool success);
    void abs_spi_cs_pin_init();
    static bool even_parity16(uint16_t value);
    static void as5047p_read_angle_cmd(uint16_t *data);
    bool abs_spi_pos_updated_ = false;
    Mode mode_ = MODE_SPI_ABS_AS5047P;
    bool abs_spi_is_discontinuous_ = true; // if true, this means an impulse on CSn is required between SPI transmission and reception
    uint16_t abs_spi_dma_tx_[1] = {0xFFFF};
    uint16_t abs_spi_dma_rx_[1];
    Stm32SpiArbiter::SpiTask spi_task_;

    constexpr float getCoggingRatio(){
        return 1.0f / 3600.0f;
    }

};

#endif // __ENCODER_HPP
