#include "app_gray_track.h"
#include "ti_msp_dl_config.h"

#define GRAY_SENSOR_ACTIVE_LOW (1U)
#define GRAY_JUNCTION_CONFIRM_SAMPLES (3U)
#define GRAY_JUNCTION_CLEAR_SAMPLES (5U)
#define GRAY_ERROR_SCALE (75)

#define GRAY_LEFT_OUTER_BIT (1U << 4)
#define GRAY_LEFT_INNER_BIT (1U << 3)
#define GRAY_CENTER_BIT (1U << 2)
#define GRAY_RIGHT_INNER_BIT (1U << 1)
#define GRAY_RIGHT_OUTER_BIT (1U << 0)

static bool gTurnArmed;
static bool gTurnPending;
static int16_t gPendingTurnAngleDeg;
static uint8_t gJunctionConfirmCount;
static uint8_t gJunctionClearCount;
static int32_t gLastError;

static bool sensor_active(GPIO_Regs *port, uint32_t pin);
static uint8_t read_sensor_bits(void);
static int32_t calculate_error(uint8_t bits, bool *valid);
static int16_t detect_turn(uint8_t bits);

void gray_track_init(void)
{
    gTurnArmed = true;
    gTurnPending = false;
    gPendingTurnAngleDeg = 0;
    gJunctionConfirmCount = 0U;
    gJunctionClearCount = 0U;
    gLastError = 0;
}

GrayTrackResult gray_track_update(uint32_t nowMs)
{
    GrayTrackResult result = {0};
    int16_t detectedTurn;

    (void) nowMs;

    result.sensor_bits = read_sensor_bits();
    result.error = calculate_error(result.sensor_bits, &result.valid);
    detectedTurn = detect_turn(result.sensor_bits);

    if (detectedTurn == 0) {
        gJunctionConfirmCount = 0U;
        if (gJunctionClearCount < GRAY_JUNCTION_CLEAR_SAMPLES) {
            gJunctionClearCount++;
        }
        if (gJunctionClearCount >= GRAY_JUNCTION_CLEAR_SAMPLES) {
            gTurnArmed = true;
        }
    } else {
        gJunctionClearCount = 0U;
        if (gTurnArmed && !gTurnPending) {
            if (gJunctionConfirmCount < GRAY_JUNCTION_CONFIRM_SAMPLES) {
                gJunctionConfirmCount++;
            }
            if (gJunctionConfirmCount >= GRAY_JUNCTION_CONFIRM_SAMPLES) {
                gTurnPending = true;
                gTurnArmed = false;
                gPendingTurnAngleDeg = detectedTurn;
            }
        }
    }

    if (gTurnPending && !result.valid) {
        result.turn_angle_deg = gPendingTurnAngleDeg;
        gTurnPending = false;
        gPendingTurnAngleDeg = 0;
    }

    return result;
}

uint8_t gray_track_get_level_bits(void)
{
    return (uint8_t) (~read_sensor_bits()) & 0x1FU;
}

static bool sensor_active(GPIO_Regs *port, uint32_t pin)
{
    bool high = (DL_GPIO_readPins(port, pin) & pin) != 0U;
#if GRAY_SENSOR_ACTIVE_LOW
    return !high;
#else
    return high;
#endif
}

static uint8_t read_sensor_bits(void)
{
    uint8_t bits = 0U;

    if (sensor_active(GPIO_GRAY_TRACK_PORT,
            GPIO_GRAY_TRACK_GRAY_LEFT_OUTER_PIN)) {
        bits |= GRAY_LEFT_OUTER_BIT;
    }
    if (sensor_active(GPIO_GRAY_TRACK_PORT,
            GPIO_GRAY_TRACK_GRAY_LEFT_INNER_PIN)) {
        bits |= GRAY_LEFT_INNER_BIT;
    }
    if (sensor_active(GPIO_GRAY_TRACK_PORT,
            GPIO_GRAY_TRACK_GRAY_CENTER_PIN)) {
        bits |= GRAY_CENTER_BIT;
    }
    if (sensor_active(GPIO_GRAY_TRACK_B_PORT,
            GPIO_GRAY_TRACK_B_GRAY_RIGHT_INNER_PIN)) {
        bits |= GRAY_RIGHT_INNER_BIT;
    }
    if (sensor_active(GPIO_GRAY_TRACK_B_PORT,
            GPIO_GRAY_TRACK_B_GRAY_RIGHT_OUTER_PIN)) {
        bits |= GRAY_RIGHT_OUTER_BIT;
    }

    return bits;
}

static int32_t calculate_error(uint8_t bits, bool *valid)
{
    static const int8_t weights[5] = {-2, -1, 0, 1, 2};
    static const uint8_t masks[5] = {
        GRAY_LEFT_OUTER_BIT, GRAY_LEFT_INNER_BIT, GRAY_CENTER_BIT,
        GRAY_RIGHT_INNER_BIT, GRAY_RIGHT_OUTER_BIT
    };
    int32_t weightedSum = 0;
    int32_t activeCount = 0;

    for (uint32_t index = 0U; index < 5U; index++) {
        if ((bits & masks[index]) != 0U) {
            weightedSum += weights[index];
            activeCount++;
        }
    }

    *valid = activeCount > 0;
    if (!*valid) {
        return gLastError;
    }

    gLastError = (weightedSum * GRAY_ERROR_SCALE) / activeCount;
    return gLastError;
}

static int16_t detect_turn(uint8_t bits)
{
    bool leftBranch = (bits & (GRAY_LEFT_OUTER_BIT | GRAY_LEFT_INNER_BIT)) ==
        (GRAY_LEFT_OUTER_BIT | GRAY_LEFT_INNER_BIT);
    bool rightBranch =
        (bits & (GRAY_RIGHT_INNER_BIT | GRAY_RIGHT_OUTER_BIT)) ==
        (GRAY_RIGHT_INNER_BIT | GRAY_RIGHT_OUTER_BIT);

    if (leftBranch && !rightBranch) {
        return 90;
    }
    if (rightBranch && !leftBranch) {
        return -90;
    }

    return 0;
}
