
#include "cordic.h"
#include <utils.hpp>

constexpr float MOD = static_cast<float>(0x80000000U);
constexpr float MOD_DIV_BY_PI = MOD / M_PI;
constexpr float INV_MOD = 1.0f / MOD;

void cordic_config() {
    CORDIC_ConfigTypeDef config;
    config.Function = CORDIC_FUNCTION_COSINE;
    config.Precision = CORDIC_PRECISION_6CYCLES;
    config.Scale = CORDIC_SCALE_0;
    config.NbWrite = CORDIC_NBWRITE_1;
    config.NbRead = CORDIC_NBREAD_2;
    config.InSize = CORDIC_INSIZE_32BITS;
    config.OutSize = CORDIC_OUTSIZE_32BITS;

    HAL_CORDIC_Configure(&hcordic, &config);
}

/**
 * @brief Calculates the cosine and sine of an angle using the CORDIC hardware accelerator
 * 
 * This function uses the STM32's CORDIC hardware accelerator to efficiently compute
 * both sine and cosine values for a given angle. The angle is first normalized to
 * [-π, π] range and then converted to Q1.31 fixed-point format required by CORDIC.
 * 
 * @param angle Input angle in radians
 * @param cos Pointer to store the calculated cosine value in range [-1.0, 1.0]
 * @param sin Pointer to store the calculated sine value in range [-1.0, 1.0]
 * 
 * @note Requires prior initialization of CORDIC peripheral using cordic_config()
 * @note Uses Data Memory Barrier (DMB) to ensure memory operation ordering
 */
void cordic_cos_sin(float angle, float* cos, float* sin) {
    float normalized_angle = wrap_pm_pi(angle);
    int32_t angle_q1_31 = static_cast<int32_t>(normalized_angle * MOD_DIV_BY_PI);
    
    CORDIC->WDATA = angle_q1_31;
    
    // Ensure that the write to WDATA is completed before reading RDATA
    __DMB();
    
    *cos = static_cast<float>((int32_t)CORDIC->RDATA) * INV_MOD;
    *sin = static_cast<float>((int32_t)CORDIC->RDATA) * INV_MOD;