#include "app_attitude.h"

#include "app_util.h"
#include "bsp/jy61/bsp_jy61.h"
#include "hw/hw_uart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ATTITUDE_BUFFER_SIZE (192U)

static bool gJy61Ready;

static void retry_jy61_init(void);
static void send_jy61_status(void);
static void send_jy61_raw_status(void);
static void send_jy61_quaternion(void);
static bool process_jy61_baud_command(const char *command);
static void scan_jy61_baudrate(void);

void app_attitude_init(void)
{
    retry_jy61_init();
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
            uart_debug_write_string("OK JY61 ZERO\r\n");
        }
        return true;
    }
    return false;
}

void app_attitude_poll(void)
{
    if (gJy61Ready) {
        jy61_poll();
    }
}

void app_attitude_send(uint32_t nowMs)
{
    Jy61Angles angles;
    char message[ATTITUDE_BUFFER_SIZE];
    int length;

    (void) nowMs;

    if (!gJy61Ready) {
        return;
    }
    if (!jy61_read_angles(&angles)) {
        return;
    }

    length = snprintf(message, sizeof(message),
        "ATT SRC=JY61 PITCH=%ld ROLL=%ld YAW=%ld\r\n",
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
