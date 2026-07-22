#include "app_attitude.h"

#include "app_util.h"
#include "bsp/jy61/bsp_jy61.h"
#include "hw/hw_uart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ATTITUDE_BUFFER_SIZE (192U)
#define ATTITUDE_PI (3.14159265358979323846f)
#define ATTITUDE_DEG_TO_RAD (ATTITUDE_PI / 180.0f)
#define ATTITUDE_RAD_TO_DEG (180.0f / ATTITUDE_PI)
#define ATTITUDE_PITCH_SIGN (-1.0f)
#define ATTITUDE_ROLL_SIGN (-1.0f)
#define ATTITUDE_YAW_SIGN (1.0f)
#define ATTITUDE_YAW_FILTER_ALPHA (0.45f)

typedef struct {
    float m[3][3];
} AttitudeMatrix;

static bool gJy61Ready;
static bool gAttitudeEnabled = true;
static AttitudeMatrix gAttitudeZeroMatrix;
static bool gAttitudeZeroValid;
static Jy61Angles gBodyAngles;
static bool gBodyAnglesValid;
static float gFilteredYawDeg;
static bool gFilteredYawValid;

static void retry_jy61_init(void);
static void send_jy61_status(void);
static void send_jy61_raw_status(void);
static void send_jy61_quaternion(void);
static bool process_jy61_baud_command(const char *command);
static void scan_jy61_baudrate(void);
static void attitude_identity_matrix(AttitudeMatrix *matrix);
static AttitudeMatrix attitude_matrix_multiply(const AttitudeMatrix *left,
    const AttitudeMatrix *right);
static AttitudeMatrix attitude_matrix_transpose(const AttitudeMatrix *matrix);
static AttitudeMatrix attitude_matrix_from_angles(const Jy61Angles *angles);
static AttitudeMatrix attitude_apply_default_mount(const AttitudeMatrix *sensor);
static Jy61Angles attitude_angles_from_matrix(const AttitudeMatrix *matrix);
static float wrap_angle(float value);

void app_attitude_init(void)
{
    gAttitudeEnabled = true;
    retry_jy61_init();
}

void app_attitude_set_enabled(bool enabled)
{
    gAttitudeEnabled = enabled;
    if (!enabled) {
        gAttitudeZeroValid = false;
        gBodyAnglesValid = false;
        gFilteredYawValid = false;
    }
}

bool app_attitude_is_enabled(void)
{
    return gAttitudeEnabled;
}

bool app_attitude_process_command(const char *command)
{
    if ((strcmp(command, "JY") == 0) || (strcmp(command, "JY61") == 0) ||
        (strcmp(command, "MPU") == 0)) {
        send_jy61_status();
        return true;
    }
    if ((strcmp(command, "JYINIT") == 0) || (strcmp(command, "MPUINIT") == 0)) {
        retry_jy61_init();
        send_jy61_status();
        return true;
    }
    if (strcmp(command, "JYQUAT") == 0) {
        send_jy61_quaternion();
        return true;
    }
    if (strcmp(command, "JYRAW") == 0) {
        send_jy61_raw_status();
        return true;
    }
    if (strcmp(command, "JYCFG") == 0) {
        if (jy61_configure_output()) {
            uart_debug_write_string("OK JY61 CFG ANGLE+QUAT\r\n");
        } else {
            uart_debug_write_string("ERR JY61 CFG\r\n");
        }
        return true;
    }
    if (strcmp(command, "JYSCAN") == 0) {
        scan_jy61_baudrate();
        return true;
    }
    if (process_jy61_baud_command(command)) {
        return true;
    }
    if ((strcmp(command, "JYZERO") == 0) || (strcmp(command, "MPUZERO") == 0)) {
        if (jy61_zero_yaw()) {
            gAttitudeZeroValid = false;
            gBodyAnglesValid = false;
            gFilteredYawValid = false;
            uart_debug_write_string("OK JY61 ZERO\r\n");
        }
        return true;
    }
    return false;
}

void app_attitude_poll(void)
{
    if (gJy61Ready && gAttitudeEnabled) {
        jy61_poll();
    }
}

bool app_attitude_read_yaw(float *yawDeg)
{
    if ((yawDeg == NULL) || !gAttitudeEnabled || !gBodyAnglesValid) {
        return false;
    }

    *yawDeg = gBodyAngles.yaw;
    return true;
}

void app_attitude_send(uint32_t nowMs)
{
    Jy61Angles angles;
    Jy61Angles bodyAngles;
    AttitudeMatrix sensorMatrix;
    AttitudeMatrix bodyMatrix;
    AttitudeMatrix relativeMatrix;
    AttitudeMatrix zeroTranspose;
    char message[ATTITUDE_BUFFER_SIZE];
    int length;

    (void) nowMs;

    if (!gJy61Ready || !gAttitudeEnabled) {
        return;
    }
    if (!jy61_read_angles(&angles)) {
        return;
    }

    sensorMatrix = attitude_matrix_from_angles(&angles);
    bodyMatrix = attitude_apply_default_mount(&sensorMatrix);
    if (!gAttitudeZeroValid) {
        gAttitudeZeroMatrix = bodyMatrix;
        gAttitudeZeroValid = true;
    }
    zeroTranspose = attitude_matrix_transpose(&gAttitudeZeroMatrix);
    relativeMatrix = attitude_matrix_multiply(&bodyMatrix, &zeroTranspose);
    bodyAngles = attitude_angles_from_matrix(&relativeMatrix);
    if (!gFilteredYawValid) {
        gFilteredYawDeg = bodyAngles.yaw;
        gFilteredYawValid = true;
    } else {
        gFilteredYawDeg = wrap_angle(gFilteredYawDeg +
            (wrap_angle(bodyAngles.yaw - gFilteredYawDeg) *
                ATTITUDE_YAW_FILTER_ALPHA));
        bodyAngles.yaw = gFilteredYawDeg;
    }
    gBodyAngles = bodyAngles;
    gBodyAnglesValid = true;

    length = snprintf(message, sizeof(message),
        "ATT SRC=JY61BODY PITCH=%ld ROLL=%ld YAW=%ld RAWP=%ld RAWR=%ld RAWY=%ld\r\n",
        (long) app_scale_float_100(bodyAngles.pitch),
        (long) app_scale_float_100(bodyAngles.roll),
        (long) app_scale_float_100(bodyAngles.yaw),
        (long) app_scale_float_100(angles.pitch),
        (long) app_scale_float_100(angles.roll),
        (long) app_scale_float_100(angles.yaw));
    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void retry_jy61_init(void)
{
    gJy61Ready = false;
    gAttitudeZeroValid = false;
    gBodyAnglesValid = false;
    gFilteredYawValid = false;
    attitude_identity_matrix(&gAttitudeZeroMatrix);

    if (jy61_init()) {
        gJy61Ready = true;
        uart_debug_write_string("OK JY61 UART\r\n");
        (void) jy61_configure_output();
    } else {
        uart_debug_write_string("ERR JY61 INIT\r\n");
    }
}

static void send_jy61_status(void)
{
    char message[ATTITUDE_BUFFER_SIZE];
    Jy61Debug debug;
    bool debugReady = jy61_read_debug(&debug);

    int length = snprintf(message, sizeof(message),
        "JY61 READY=%u UART=1 BAUD=%lu BYTE=%lu ANG=%lu QUAT=%lu SUMERR=%lu IRQ=%lu POLL=%lu TYPE=0x%02X AOK=%u QOK=%u FRAME=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X DBG=%u\r\n",
        gJy61Ready ? 1U : 0U,
        (unsigned long) jy61_get_baudrate(),
        debugReady ? (unsigned long) debug.byteCount : 0UL,
        debugReady ? (unsigned long) debug.angleFrameCount : 0UL,
        debugReady ? (unsigned long) debug.quatFrameCount : 0UL,
        debugReady ? (unsigned long) debug.checksumFailCount : 0UL,
        debugReady ? (unsigned long) debug.irqCount : 0UL,
        debugReady ? (unsigned long) debug.pollCount : 0UL,
        debugReady ? (unsigned int) debug.lastFrameType : 0U,
        (debugReady && debug.angleValid) ? 1U : 0U,
        (debugReady && debug.quatValid) ? 1U : 0U,
        debugReady ? (unsigned int) debug.lastFrame[0] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[1] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[2] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[3] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[4] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[5] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[6] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[7] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[8] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[9] : 0U,
        debugReady ? (unsigned int) debug.lastFrame[10] : 0U,
        debugReady ? 1U : 0U);

    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void send_jy61_raw_status(void)
{
    char message[ATTITUDE_BUFFER_SIZE];
    Jy61Debug debug;
    bool debugReady = jy61_read_debug(&debug);

    int length = snprintf(message, sizeof(message),
        "JYRAW BAUD=%lu RXH=%u RXE=%u RXF=%u RIS=0x%08lX MIS=0x%08lX ERR=0x%08lX BYTE=%lu IRQ=%lu POLL=%lu\r\n",
        (unsigned long) jy61_get_baudrate(),
        (debugReady && debug.rxPinHigh) ? 1U : 0U,
        (debugReady && debug.rxFifoEmpty) ? 1U : 0U,
        (debugReady && debug.rxFifoFull) ? 1U : 0U,
        debugReady ? (unsigned long) debug.rxRawStatus : 0UL,
        debugReady ? (unsigned long) debug.rxEnabledStatus : 0UL,
        debugReady ? (unsigned long) debug.errorStatus : 0UL,
        debugReady ? (unsigned long) debug.byteCount : 0UL,
        debugReady ? (unsigned long) debug.irqCount : 0UL,
        debugReady ? (unsigned long) debug.pollCount : 0UL);

    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static bool process_jy61_baud_command(const char *command)
{
    char message[ATTITUDE_BUFFER_SIZE];
    char *end;
    unsigned long baudrate;
    int length;

    if (strncmp(command, "JYBAUD ", 7U) != 0) {
        return false;
    }

    baudrate = strtoul(&command[7], &end, 10);
    if ((*end != '\0') || !jy61_set_baudrate((uint32_t) baudrate)) {
        uart_debug_write_string("ERR JY61 BAUD\r\n");
        return true;
    }

    length = snprintf(message, sizeof(message), "OK JY61 BAUD %lu\r\n",
        baudrate);
    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
    return true;
}

static void scan_jy61_baudrate(void)
{
    static const uint32_t baudrates[] = {
        9600U,
        19200U,
        38400U,
        57600U,
        115200U,
        230400U,
    };
    char message[ATTITUDE_BUFFER_SIZE];

    for (uint8_t i = 0U; i < (sizeof(baudrates) / sizeof(baudrates[0]));
         i++) {
        if (jy61_probe_baudrate(baudrates[i], 300U)) {
            int length = snprintf(message, sizeof(message),
                "OK JY61 SCAN BAUD=%lu\r\n", (unsigned long) baudrates[i]);
            if ((length > 0) && ((size_t) length < sizeof(message))) {
                uart_debug_write_string(message);
            }
            (void) jy61_configure_output();
            return;
        }
    }

    (void) jy61_set_baudrate(9600U);
    uart_debug_write_string(
        "ERR JY61 SCAN BYTE=0 CHECK TX->PB7 GND VCC\r\n");
}

static void send_jy61_quaternion(void)
{
    char message[ATTITUDE_BUFFER_SIZE];
    Jy61Quaternion quat;
    int length;

    if (!jy61_read_quaternion(&quat)) {
        uart_debug_write_string("ERR JY61 QUAT\r\n");
        return;
    }

    length = snprintf(message, sizeof(message),
        "QUAT SRC=JY61 W=%ld X=%ld Y=%ld Z=%ld\r\n",
        (long) app_scale_float_10000(quat.w),
        (long) app_scale_float_10000(quat.x),
        (long) app_scale_float_10000(quat.y),
        (long) app_scale_float_10000(quat.z));
    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void attitude_identity_matrix(AttitudeMatrix *matrix)
{
    if (matrix == NULL) {
        return;
    }

    matrix->m[0][0] = 1.0f;
    matrix->m[0][1] = 0.0f;
    matrix->m[0][2] = 0.0f;
    matrix->m[1][0] = 0.0f;
    matrix->m[1][1] = 1.0f;
    matrix->m[1][2] = 0.0f;
    matrix->m[2][0] = 0.0f;
    matrix->m[2][1] = 0.0f;
    matrix->m[2][2] = 1.0f;
}

static AttitudeMatrix attitude_matrix_multiply(const AttitudeMatrix *left,
    const AttitudeMatrix *right)
{
    AttitudeMatrix result;

    for (uint8_t row = 0U; row < 3U; row++) {
        for (uint8_t col = 0U; col < 3U; col++) {
            result.m[row][col] =
                (left->m[row][0] * right->m[0][col]) +
                (left->m[row][1] * right->m[1][col]) +
                (left->m[row][2] * right->m[2][col]);
        }
    }

    return result;
}

static AttitudeMatrix attitude_matrix_transpose(const AttitudeMatrix *matrix)
{
    AttitudeMatrix result;

    for (uint8_t row = 0U; row < 3U; row++) {
        for (uint8_t col = 0U; col < 3U; col++) {
            result.m[row][col] = matrix->m[col][row];
        }
    }

    return result;
}

static AttitudeMatrix attitude_matrix_from_angles(const Jy61Angles *angles)
{
    AttitudeMatrix pitchMatrix;
    AttitudeMatrix yawMatrix;
    AttitudeMatrix rollMatrix;
    AttitudeMatrix yawPitchMatrix;
    float pitch = angles->pitch * ATTITUDE_PITCH_SIGN * ATTITUDE_DEG_TO_RAD;
    float roll = angles->roll * ATTITUDE_ROLL_SIGN * ATTITUDE_DEG_TO_RAD;
    float yaw = angles->yaw * ATTITUDE_YAW_SIGN * ATTITUDE_DEG_TO_RAD;
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    pitchMatrix.m[0][0] = 1.0f;
    pitchMatrix.m[0][1] = 0.0f;
    pitchMatrix.m[0][2] = 0.0f;
    pitchMatrix.m[1][0] = 0.0f;
    pitchMatrix.m[1][1] = cp;
    pitchMatrix.m[1][2] = -sp;
    pitchMatrix.m[2][0] = 0.0f;
    pitchMatrix.m[2][1] = sp;
    pitchMatrix.m[2][2] = cp;

    yawMatrix.m[0][0] = cy;
    yawMatrix.m[0][1] = 0.0f;
    yawMatrix.m[0][2] = sy;
    yawMatrix.m[1][0] = 0.0f;
    yawMatrix.m[1][1] = 1.0f;
    yawMatrix.m[1][2] = 0.0f;
    yawMatrix.m[2][0] = -sy;
    yawMatrix.m[2][1] = 0.0f;
    yawMatrix.m[2][2] = cy;

    rollMatrix.m[0][0] = cr;
    rollMatrix.m[0][1] = -sr;
    rollMatrix.m[0][2] = 0.0f;
    rollMatrix.m[1][0] = sr;
    rollMatrix.m[1][1] = cr;
    rollMatrix.m[1][2] = 0.0f;
    rollMatrix.m[2][0] = 0.0f;
    rollMatrix.m[2][1] = 0.0f;
    rollMatrix.m[2][2] = 1.0f;

    yawPitchMatrix = attitude_matrix_multiply(&yawMatrix, &pitchMatrix);
    return attitude_matrix_multiply(&rollMatrix, &yawPitchMatrix);
}

static AttitudeMatrix attitude_apply_default_mount(const AttitudeMatrix *sensor)
{
    AttitudeMatrix mountMatrix;

    /* Default physical install: vehicle front = sensor -X, vehicle top = +Y. */
    mountMatrix.m[0][0] = 0.0f;
    mountMatrix.m[0][1] = 0.0f;
    mountMatrix.m[0][2] = 1.0f;
    mountMatrix.m[1][0] = 0.0f;
    mountMatrix.m[1][1] = 1.0f;
    mountMatrix.m[1][2] = 0.0f;
    mountMatrix.m[2][0] = -1.0f;
    mountMatrix.m[2][1] = 0.0f;
    mountMatrix.m[2][2] = 0.0f;

    return attitude_matrix_multiply(&mountMatrix, sensor);
}

static Jy61Angles attitude_angles_from_matrix(const AttitudeMatrix *matrix)
{
    Jy61Angles angles;
    float pitch = asinf(app_clamp_float(matrix->m[2][1], -1.0f, 1.0f));
    float cp = cosf(pitch);
    float yaw;
    float roll;

    if (app_abs_float(cp) > 0.0001f) {
        yaw = atan2f(-matrix->m[2][0], matrix->m[2][2]);
        roll = atan2f(-matrix->m[0][1], matrix->m[1][1]);
    } else {
        yaw = atan2f(matrix->m[0][2], matrix->m[0][0]);
        roll = 0.0f;
    }

    angles.pitch = wrap_angle((pitch * ATTITUDE_RAD_TO_DEG) /
        ATTITUDE_PITCH_SIGN);
    angles.roll = wrap_angle((roll * ATTITUDE_RAD_TO_DEG) /
        ATTITUDE_ROLL_SIGN);
    angles.yaw = wrap_angle((yaw * ATTITUDE_RAD_TO_DEG) /
        ATTITUDE_YAW_SIGN);
    return angles;
}

static float wrap_angle(float value)
{
    while (value > 180.0f) {
        value -= 360.0f;
    }
    while (value < -180.0f) {
        value += 360.0f;
    }
    return value;
}
