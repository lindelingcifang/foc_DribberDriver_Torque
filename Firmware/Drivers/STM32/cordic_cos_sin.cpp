
// #include "cordic.h"  // CORDIC temporarily unavailable
#include <utils.hpp>
#include <cstdint>

// 1/(2*PI) for angle normalization
static constexpr float INV_TWO_PI = 0.159154943092f;

// Lookup table size (must be power of 2)
static constexpr uint16_t SIN_TABLE_SIZE = 256;
static constexpr uint16_t SIN_TABLE_MASK = SIN_TABLE_SIZE - 1;

// Precomputed sine lookup table [0, 2*PI) with 256 entries
// sin(i * 2*PI / 256) for i = 0..256
static const float sinTable[SIN_TABLE_SIZE + 1] = {
    0.000000000f, 0.024541229f, 0.049067674f, 0.073564564f, 0.098017140f, 0.122410675f, 0.146730474f, 0.170961889f,
    0.195090322f, 0.219101240f, 0.242980180f, 0.266712757f, 0.290284677f, 0.313681740f, 0.336889853f, 0.359895037f,
    0.382683432f, 0.405241314f, 0.427555093f, 0.449611330f, 0.471396737f, 0.492898192f, 0.514102744f, 0.534997620f,
    0.555570233f, 0.575808191f, 0.595699304f, 0.615231591f, 0.634393284f, 0.653172843f, 0.671558955f, 0.689540545f,
    0.707106781f, 0.724247083f, 0.740951125f, 0.757208847f, 0.773010453f, 0.788346428f, 0.803207531f, 0.817584813f,
    0.831469612f, 0.844853565f, 0.857728610f, 0.870086991f, 0.881921264f, 0.893224301f, 0.903989293f, 0.914209756f,
    0.923879533f, 0.932992799f, 0.941544065f, 0.949528181f, 0.956940336f, 0.963776066f, 0.970031253f, 0.975702130f,
    0.980785280f, 0.985277642f, 0.989176510f, 0.992479535f, 0.995184727f, 0.997290457f, 0.998795456f, 0.999698819f,
    1.000000000f, 0.999698819f, 0.998795456f, 0.997290457f, 0.995184727f, 0.992479535f, 0.989176510f, 0.985277642f,
    0.980785280f, 0.975702130f, 0.970031253f, 0.963776066f, 0.956940336f, 0.949528181f, 0.941544065f, 0.932992799f,
    0.923879533f, 0.914209756f, 0.903989293f, 0.893224301f, 0.881921264f, 0.870086991f, 0.857728610f, 0.844853565f,
    0.831469612f, 0.817584813f, 0.803207531f, 0.788346428f, 0.773010453f, 0.757208847f, 0.740951125f, 0.724247083f,
    0.707106781f, 0.689540545f, 0.671558955f, 0.653172843f, 0.634393284f, 0.615231591f, 0.595699304f, 0.575808191f,
    0.555570233f, 0.534997620f, 0.514102744f, 0.492898192f, 0.471396737f, 0.449611330f, 0.427555093f, 0.405241314f,
    0.382683432f, 0.359895037f, 0.336889853f, 0.313681740f, 0.290284677f, 0.266712757f, 0.242980180f, 0.219101240f,
    0.195090322f, 0.170961889f, 0.146730474f, 0.122410675f, 0.098017140f, 0.073564564f, 0.049067674f, 0.024541229f,
    0.000000000f, -0.024541229f, -0.049067674f, -0.073564564f, -0.098017140f, -0.122410675f, -0.146730474f, -0.170961889f,
    -0.195090322f, -0.219101240f, -0.242980180f, -0.266712757f, -0.290284677f, -0.313681740f, -0.336889853f, -0.359895037f,
    -0.382683432f, -0.405241314f, -0.427555093f, -0.449611330f, -0.471396737f, -0.492898192f, -0.514102744f, -0.534997620f,
    -0.555570233f, -0.575808191f, -0.595699304f, -0.615231591f, -0.634393284f, -0.653172843f, -0.671558955f, -0.689540545f,
    -0.707106781f, -0.724247083f, -0.740951125f, -0.757208847f, -0.773010453f, -0.788346428f, -0.803207531f, -0.817584813f,
    -0.831469612f, -0.844853565f, -0.857728610f, -0.870086991f, -0.881921264f, -0.893224301f, -0.903989293f, -0.914209756f,
    -0.923879533f, -0.932992799f, -0.941544065f, -0.949528181f, -0.956940336f, -0.963776066f, -0.970031253f, -0.975702130f,
    -0.980785280f, -0.985277642f, -0.989176510f, -0.992479535f, -0.995184727f, -0.997290457f, -0.998795456f, -0.999698819f,
    -1.000000000f, -0.999698819f, -0.998795456f, -0.997290457f, -0.995184727f, -0.992479535f, -0.989176510f, -0.985277642f,
    -0.980785280f, -0.975702130f, -0.970031253f, -0.963776066f, -0.956940336f, -0.949528181f, -0.941544065f, -0.932992799f,
    -0.923879533f, -0.914209756f, -0.903989293f, -0.893224301f, -0.881921264f, -0.870086991f, -0.857728610f, -0.844853565f,
    -0.831469612f, -0.817584813f, -0.803207531f, -0.788346428f, -0.773010453f, -0.757208847f, -0.740951125f, -0.724247083f,
    -0.707106781f, -0.689540545f, -0.671558955f, -0.653172843f, -0.634393284f, -0.615231591f, -0.595699304f, -0.575808191f,
    -0.555570233f, -0.534997620f, -0.514102744f, -0.492898192f, -0.471396737f, -0.449611330f, -0.427555093f, -0.405241314f,
    -0.382683432f, -0.359895037f, -0.336889853f, -0.313681740f, -0.290284677f, -0.266712757f, -0.242980180f, -0.219101240f,
    -0.195090322f, -0.170961889f, -0.146730474f, -0.122410675f, -0.098017140f, -0.073564564f, -0.049067674f, -0.024541229f,
    0.000000000f  // wrap-around entry for interpolation
};

void cordic_config() {
    // CORDIC temporarily unavailable, using software lookup table instead
    // No initialization needed for software implementation
}

/**
 * @brief Calculates the cosine and sine of an angle using lookup table with linear interpolation
 * 
 * This function uses a precomputed sine lookup table with linear interpolation to compute
 * both sine and cosine values for a given angle. Optimized for speed over precision.
 * 
 * @param angle Input angle in radians
 * @param cos Pointer to store the calculated cosine value in range [-1.0, 1.0]
 * @param sin Pointer to store the calculated sine value in range [-1.0, 1.0]
 * 
 * @note Uses 256-entry lookup table with linear interpolation
 * @note Accuracy: approximately 3-4 decimal places
 */
void cordic_cos_sin(float angle, float* cos, float* sin) {
    float in, findex, fract;
    uint16_t indexS, indexC;
    int32_t n;

    // Scale input from radians to [0, 1] range: in = angle / (2*PI)
    in = angle * INV_TWO_PI;

    // Calculate floor value
    n = static_cast<int32_t>(in);
    if (in < 0.0f) {
        n--;
    }
    
    // Map to [0, 1] range
    in = in - static_cast<float>(n);

    // Calculate table index for sine
    findex = static_cast<float>(SIN_TABLE_SIZE) * in;
    indexS = static_cast<uint16_t>(findex) & SIN_TABLE_MASK;

    // Cosine index is 90 degrees (64 entries) ahead of sine
    indexC = (indexS + (SIN_TABLE_SIZE / 4)) & SIN_TABLE_MASK;

    // Fractional part for interpolation
    fract = findex - static_cast<float>(static_cast<uint16_t>(findex));

    // Linear interpolation for sine: sin = a + fract * (b - a)
    *sin = sinTable[indexS] + fract * (sinTable[indexS + 1] - sinTable[indexS]);

    // Linear interpolation for cosine
    *cos = sinTable[indexC] + fract * (sinTable[indexC + 1] - sinTable[indexC]);
}