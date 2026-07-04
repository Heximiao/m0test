#include "app_mpu6050_attitude.h"

#include "app_util.h"
#include "bsp/mpu6050/bsp_mpu6050.h"
#include "hw/hw_uart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ATTITUDE_BUFFER_SIZE (192U)
#define DMP_INIT_ATTEMPTS (5U)
#define DMP_INIT_RETRY_DELAY_MS (200U)
#define DMP_SETTLE_DELAY_MS (200U)

static bool gMpu6050Ready;
static uint32_t gDmpReadFailCount;
static uint32_t gAccelFallbackCount;
static uint32_t gAccelReadFailCount;

static void retry_mpu6050_init(void);
static void send_mpu_status(void);

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
    if (strcmp(command, "MPUZERO") == 0) {
        mpu6050_reset_fallback_angles();
        uart_debug_write_string("OK MPU ZERO\r\n");
        return true;
    }
    return false;
}

void app_mpu6050_attitude_send(uint32_t nowMs)
{
    Mpu6050Angles angles;
    char message[ATTITUDE_BUFFER_SIZE];
    const char *source = "DMP";
    int length;

    if (!gMpu6050Ready) {
        return;
    }
    if (!mpu6050_read_angles(&angles)) {
        gDmpReadFailCount++;
        if (!mpu6050_read_fallback_angles(&angles, nowMs)) {
            gAccelReadFailCount++;
            return;
        }
        gAccelFallbackCount++;
        source = "IMU";
    }

    length = snprintf(message, sizeof(message),
        "ATT SRC=%s PITCH=%ld ROLL=%ld YAW=%ld\r\n",
        source,
        (long) app_scale_float_100(angles.pitch),
        (long) app_scale_float_100(angles.roll),
        (long) app_scale_float_100(angles.yaw));
    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void retry_mpu6050_init(void)
{
    gMpu6050Ready = false;
    gDmpReadFailCount = 0U;
    gAccelFallbackCount = 0U;
    gAccelReadFailCount = 0U;
    mpu6050_reset_fallback_angles();

    if (mpu6050_init()) {
        uart_debug_write_string("MPU6050 detected, initializing DMP\r\n");
        for (uint8_t attempt = 1U; attempt <= DMP_INIT_ATTEMPTS; attempt++) {
            if (mpu6050_dmp_init()) {
                mpu6050_delay_ms(DMP_SETTLE_DELAY_MS);
                gMpu6050Ready = true;
                uart_debug_write_string("OK MPU6050 DMP\r\n");
                return;
            }
            uart_debug_write_string("ERR MPU6050 DMP RETRY\r\n");
            mpu6050_release_bus();
            mpu6050_delay_ms(DMP_INIT_RETRY_DELAY_MS);
        }
        uart_debug_write_string("ERR MPU6050 DMP\r\n");
    } else {
        uart_debug_write_string("ERR MPU6050 INIT\r\n");
    }
}

static void send_mpu_status(void)
{
    char message[ATTITUDE_BUFFER_SIZE];
    Mpu6050Debug debug;
    uint8_t id = mpu6050_read_id();
    bool debugReady = mpu6050_read_debug(&debug);
    bool sclHigh;
    bool sdaHigh;

    mpu6050_release_bus();
    mpu6050_delay_us(10U);
    sclHigh = mpu6050_scl_is_high();
    sdaHigh = mpu6050_sda_is_high();

    int length = snprintf(message, sizeof(message),
        "MPU READY=%u ID=0x%02X SCL=%u SDA=%u DMPFAIL=%lu ACCFALL=%lu ACCFAIL=%lu DMPINIT=%u DMPERR=%u FIFO=%u INT=0x%02X UCTRL=0x%02X FEN=0x%02X DBG=%u\r\n",
        gMpu6050Ready ? 1U : 0U, (unsigned int) id,
        sclHigh ? 1U : 0U,
        sdaHigh ? 1U : 0U,
        (unsigned long) gDmpReadFailCount,
        (unsigned long) gAccelFallbackCount,
        (unsigned long) gAccelReadFailCount,
        debugReady ? (unsigned int) debug.dmpInitError : 255U,
        debugReady ? (unsigned int) debug.dmpError : 255U,
        debugReady ? (unsigned int) debug.fifoCount : 0U,
        debugReady ? (unsigned int) debug.intStatus : 0U,
        debugReady ? (unsigned int) debug.userCtrl : 0U,
        debugReady ? (unsigned int) debug.fifoEnable : 0U,
        debugReady ? 1U : 0U);

    if ((length > 0) && ((size_t) length < sizeof(message))) {
        uart_debug_write_string(message);
    }
}
