#include "app_mpu6050_attitude.h"

#include "bsp/mpu6050/bsp_mpu6050.h"
#include "hw/hw_uart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ATTITUDE_BUFFER_SIZE (96U)

static bool gMpu6050Ready;
static uint32_t gDmpReadFailCount;
static uint32_t gAccelFallbackCount;
static uint32_t gAccelReadFailCount;

static void retry_mpu6050_init(void);
static void send_mpu_status(void);
static int32_t scale_float_100(float value);

void app_mpu6050_attitude_init(void)
{
    retry_mpu6050_init();
}

bool app_mpu6050_attitude_process_command(const char *command)
{
    if (strcmp(command, "MPU") == 0) {
        send_mpu_status();
        return true;
    }
    if (strcmp(command, "MPUINIT") == 0) {
        retry_mpu6050_init();
        send_mpu_status();
        return true;
    }
    return false;
}

void app_mpu6050_attitude_send(void)
{
    Mpu6050Angles angles;
    char message[ATTITUDE_BUFFER_SIZE];
    int length;

    if (!gMpu6050Ready) {
        return;
    }
    if (!mpu6050_read_angles(&angles)) {
        gDmpReadFailCount++;
        if (!mpu6050_read_accel_angles(&angles)) {
            gAccelReadFailCount++;
            return;
        }
        gAccelFallbackCount++;
    }

    length = snprintf(message, sizeof(message),
        "ATT PITCH=%ld ROLL=%ld YAW=%ld\r\n",
        (long) scale_float_100(angles.pitch),
        (long) scale_float_100(angles.roll),
        (long) scale_float_100(angles.yaw));
    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void retry_mpu6050_init(void)
{
    if (mpu6050_init()) {
        uart_debug_write_string("MPU6050 detected, initializing DMP\r\n");
        gMpu6050Ready = mpu6050_dmp_init();
        uart_debug_write_string(gMpu6050Ready ? "OK MPU6050 DMP\r\n" :
                                               "ERR MPU6050 DMP\r\n");
    } else {
        gMpu6050Ready = false;
        uart_debug_write_string("ERR MPU6050 INIT\r\n");
    }
}

static void send_mpu_status(void)
{
    char message[ATTITUDE_BUFFER_SIZE];
    uint8_t id = mpu6050_read_id();
    mpu6050_release_bus();
    int length = snprintf(message, sizeof(message),
        "MPU READY=%u ID=0x%02X SCL=%u SDA=%u DMPFAIL=%lu ACCFALL=%lu ACCFAIL=%lu\r\n",
        gMpu6050Ready ? 1U : 0U, (unsigned int) id,
        mpu6050_scl_is_high() ? 1U : 0U,
        mpu6050_sda_is_high() ? 1U : 0U,
        (unsigned long) gDmpReadFailCount,
        (unsigned long) gAccelFallbackCount,
        (unsigned long) gAccelReadFailCount);

    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static int32_t scale_float_100(float value)
{
    if (value >= 0.0f) {
        return (int32_t) ((value * 100.0f) + 0.5f);
    }
    return (int32_t) ((value * 100.0f) - 0.5f);
}
