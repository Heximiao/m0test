#ifndef APP_UTIL_H
#define APP_UTIL_H

#include <stdint.h>

static inline float app_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static inline float app_clamp_float(float value, float minValue,
    float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }

    return value;
}

static inline int32_t app_scale_float_100(float value)
{
    if (value >= 0.0f) {
        return (int32_t) ((value * 100.0f) + 0.5f);
    }

    return (int32_t) ((value * 100.0f) - 0.5f);
}

static inline int32_t app_scale_float_1000(float value)
{
    if (value >= 0.0f) {
        return (int32_t) ((value * 1000.0f) + 0.5f);
    }

    return (int32_t) ((value * 1000.0f) - 0.5f);
}

#endif
