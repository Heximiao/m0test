#include "bsp_jy61.h"

#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <string.h>

#define JY61_FRAME_HEAD (0x55U)
#define JY61_FRAME_ANGLE (0x53U)
#define JY61_FRAME_QUATERNION (0x59U)
#define JY61_PAYLOAD_SIZE (8U)
#define JY61_FRAME_SIZE (11U)
#define JY61_DEFAULT_BAUDRATE (9600U)
#define JY61_COMMAND_DELAY_MS (200U)
#define JY61_ZERO_DELAY_MS (100U)
#define JY61_REG_SAVE (0x00U)
#define JY61_REG_CALSW (0x01U)
#define JY61_REG_RSW (0x02U)
#define JY61_REG_READADDR (0x27U)
#define JY61_REG_POWONSEND (0x2DU)
#define JY61_REG_KEY (0x69U)
#define JY61_KEY_LOW (0x88U)
#define JY61_KEY_HIGH (0xB5U)
#define JY61_RSW_ANGLE_QUAT_LOW (0x08U)
#define JY61_RSW_ANGLE_QUAT_HIGH (0x02U)

typedef enum {
    JY61_WAIT_HEAD = 0,
    JY61_WAIT_TYPE,
    JY61_WAIT_PAYLOAD,
} Jy61RxState;

static volatile Jy61RxState gRxState;
static volatile uint8_t gFrameType;
static volatile uint8_t gPayloadIndex;
static uint8_t gPayload[9];
static uint8_t gLastFrame[JY61_FRAME_SIZE];
static volatile int16_t gRollRaw;
static volatile int16_t gPitchRaw;
static volatile int16_t gYawRaw;
static volatile int16_t gQuatRaw[4];
static volatile uint32_t gByteCount;
static volatile uint32_t gAngleFrameCount;
static volatile uint32_t gQuatFrameCount;
static volatile uint32_t gChecksumFailCount;
static volatile uint32_t gIrqCount;
static volatile uint32_t gPollCount;
static volatile uint32_t gBaudRate = JY61_DEFAULT_BAUDRATE;
static volatile uint8_t gLastFrameType;
static volatile bool gAngleValid;
static volatile bool gQuatValid;

static void process_byte(uint8_t data);
static void process_frame(void);
static int16_t make_i16(uint8_t low, uint8_t high);
static float angle_from_raw(int16_t raw);
static float quat_from_raw(int16_t raw);
static void send_command5(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,
    uint8_t b4);
static void write_register(uint8_t address, uint8_t low, uint8_t high);
static void unlock_registers(void);
static void save_registers(void);
static void delay_ms(uint32_t ms);
static void poll_for_ms(uint32_t ms);
static void reset_parser(void);

bool jy61_init(void)
{
    (void) jy61_set_baudrate(JY61_DEFAULT_BAUDRATE);
    reset_parser();

    DL_UART_Main_clearInterruptStatus(UART_JY61P_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_JY61P_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_JY61P_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_JY61P_INST_INT_IRQN);
    return true;
}

bool jy61_configure_output(void)
{
    unlock_registers();
    delay_ms(JY61_COMMAND_DELAY_MS);
    write_register(JY61_REG_RSW, JY61_RSW_ANGLE_QUAT_LOW,
        JY61_RSW_ANGLE_QUAT_HIGH);
    delay_ms(JY61_COMMAND_DELAY_MS);
    write_register(JY61_REG_POWONSEND, 0x01U, 0x00U);
    delay_ms(JY61_COMMAND_DELAY_MS);
    save_registers();
    delay_ms(JY61_COMMAND_DELAY_MS);
    write_register(JY61_REG_READADDR, 0x3DU, 0x00U);
    return true;
}

bool jy61_read_angles(Jy61Angles *angles)
{
    if ((angles == NULL) || !gAngleValid) {
        return false;
    }

    angles->roll = angle_from_raw(gRollRaw);
    angles->pitch = angle_from_raw(gPitchRaw);
    angles->yaw = angle_from_raw(gYawRaw);
    return true;
}

bool jy61_read_quaternion(Jy61Quaternion *quat)
{
    if ((quat == NULL) || !gQuatValid) {
        return false;
    }

    quat->w = quat_from_raw(gQuatRaw[0]);
    quat->x = quat_from_raw(gQuatRaw[1]);
    quat->y = quat_from_raw(gQuatRaw[2]);
    quat->z = quat_from_raw(gQuatRaw[3]);
    return true;
}

bool jy61_zero_yaw(void)
{
    unlock_registers();
    delay_ms(JY61_ZERO_DELAY_MS);
    write_register(JY61_REG_CALSW, 0x04U, 0x00U);
    delay_ms(JY61_ZERO_DELAY_MS);
    save_registers();
    return true;
}

bool jy61_read_debug(Jy61Debug *debug)
{
    if (debug == NULL) {
        return false;
    }

    debug->byteCount = gByteCount;
    debug->angleFrameCount = gAngleFrameCount;
    debug->quatFrameCount = gQuatFrameCount;
    debug->checksumFailCount = gChecksumFailCount;
    debug->irqCount = gIrqCount;
    debug->pollCount = gPollCount;
    debug->rxRawStatus = DL_UART_Main_getRawInterruptStatus(UART_JY61P_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    debug->rxEnabledStatus = DL_UART_Main_getEnabledInterruptStatus(
        UART_JY61P_INST, DL_UART_MAIN_INTERRUPT_RX);
    debug->errorStatus = DL_UART_Main_getErrorStatus(UART_JY61P_INST,
        DL_UART_MAIN_ERROR_OVERRUN | DL_UART_MAIN_ERROR_BREAK |
        DL_UART_MAIN_ERROR_PARITY | DL_UART_MAIN_ERROR_FRAMING);
    debug->lastFrameType = gLastFrameType;
    debug->rxPinHigh = (DL_GPIO_readPins(GPIO_UART_JY61P_RX_PORT,
                            GPIO_UART_JY61P_RX_PIN) != 0U);
    debug->rxFifoEmpty = DL_UART_Main_isRXFIFOEmpty(UART_JY61P_INST);
    debug->rxFifoFull = DL_UART_Main_isRXFIFOFull(UART_JY61P_INST);
    debug->angleValid = gAngleValid;
    debug->quatValid = gQuatValid;
    memcpy(debug->lastFrame, gLastFrame, sizeof(debug->lastFrame));
    return true;
}

bool jy61_set_baudrate(uint32_t baudrate)
{
    uint32_t denominator;
    uint32_t integerDivisor;
    uint32_t fractionalDivisor;
    uint32_t remainder;

    if ((baudrate != 9600U) && (baudrate != 19200U) &&
        (baudrate != 38400U) && (baudrate != 57600U) &&
        (baudrate != 115200U) && (baudrate != 230400U)) {
        return false;
    }

    denominator = 16U * baudrate;
    integerDivisor = UART_JY61P_INST_FREQUENCY / denominator;
    remainder = UART_JY61P_INST_FREQUENCY % denominator;
    fractionalDivisor = ((remainder * 64U) + (denominator / 2U)) /
        denominator;
    if (fractionalDivisor >= 64U) {
        integerDivisor++;
        fractionalDivisor = 0U;
    }

    DL_UART_Main_disable(UART_JY61P_INST);
    DL_UART_Main_setOversampling(UART_JY61P_INST,
        DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_JY61P_INST, integerDivisor,
        fractionalDivisor);
    DL_UART_Main_enable(UART_JY61P_INST);

    reset_parser();
    gBaudRate = baudrate;
    return true;
}

bool jy61_probe_baudrate(uint32_t baudrate, uint32_t listenMs)
{
    uint32_t startAngleFrames;
    uint32_t startQuatFrames;

    if (!jy61_set_baudrate(baudrate)) {
        return false;
    }

    startAngleFrames = gAngleFrameCount;
    startQuatFrames = gQuatFrameCount;
    gLastFrameType = 0U;
    write_register(JY61_REG_READADDR, 0x3DU, 0x00U);
    poll_for_ms(listenMs);
    return (gLastFrameType != 0U) || (gAngleFrameCount != startAngleFrames) ||
        (gQuatFrameCount != startQuatFrames);
}

uint32_t jy61_get_baudrate(void)
{
    return gBaudRate;
}

void jy61_poll(void)
{
    uint8_t rxByte;

    while (DL_UART_receiveDataCheck(UART_JY61P_INST, &rxByte)) {
        gPollCount++;
        process_byte(rxByte);
    }
}

void jy61_handle_irq(void)
{
    uint8_t rxByte;

    gIrqCount++;

    switch (DL_UART_Main_getPendingInterrupt(UART_JY61P_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_receiveDataCheck(UART_JY61P_INST, &rxByte)) {
                process_byte(rxByte);
            }
            break;
        default:
            break;
    }
}

static void process_byte(uint8_t data)
{
    gByteCount++;

    switch (gRxState) {
        case JY61_WAIT_HEAD:
            if (data == JY61_FRAME_HEAD) {
                gRxState = JY61_WAIT_TYPE;
            }
            break;
        case JY61_WAIT_TYPE:
            if (data == JY61_FRAME_HEAD) {
                gRxState = JY61_WAIT_TYPE;
            } else {
                gFrameType = data;
                gPayloadIndex = 0U;
                gRxState = JY61_WAIT_PAYLOAD;
            }
            break;
        case JY61_WAIT_PAYLOAD:
            gPayload[gPayloadIndex++] = data;
            if (gPayloadIndex >= sizeof(gPayload)) {
                process_frame();
                gRxState = JY61_WAIT_HEAD;
            }
            break;
        default:
            gRxState = JY61_WAIT_HEAD;
            break;
    }
}

static void process_frame(void)
{
    uint8_t sum = (uint8_t) (JY61_FRAME_HEAD + gFrameType);

    for (uint8_t i = 0U; i < JY61_PAYLOAD_SIZE; i++) {
        sum = (uint8_t) (sum + gPayload[i]);
    }
    if (sum != gPayload[JY61_PAYLOAD_SIZE]) {
        gChecksumFailCount++;
        return;
    }

    gLastFrame[0] = JY61_FRAME_HEAD;
    gLastFrame[1] = gFrameType;
    memcpy(&gLastFrame[2], gPayload, sizeof(gPayload));
    gLastFrameType = gFrameType;

    if (gFrameType == JY61_FRAME_ANGLE) {
        gRollRaw = make_i16(gPayload[0], gPayload[1]);
        gPitchRaw = make_i16(gPayload[2], gPayload[3]);
        gYawRaw = make_i16(gPayload[4], gPayload[5]);
        gAngleValid = true;
        gAngleFrameCount++;
    } else if (gFrameType == JY61_FRAME_QUATERNION) {
        gQuatRaw[0] = make_i16(gPayload[0], gPayload[1]);
        gQuatRaw[1] = make_i16(gPayload[2], gPayload[3]);
        gQuatRaw[2] = make_i16(gPayload[4], gPayload[5]);
        gQuatRaw[3] = make_i16(gPayload[6], gPayload[7]);
        gQuatValid = true;
        gQuatFrameCount++;
    }
}

static int16_t make_i16(uint8_t low, uint8_t high)
{
    return (int16_t) (((uint16_t) high << 8U) | low);
}

static float angle_from_raw(int16_t raw)
{
    return ((float) raw * 180.0f) / 32768.0f;
}

static float quat_from_raw(int16_t raw)
{
    return (float) raw / 32768.0f;
}

static void send_command5(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,
    uint8_t b4)
{
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, b0);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, b1);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, b2);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, b3);
    DL_UART_Main_transmitDataBlocking(UART_JY61P_INST, b4);
}

static void write_register(uint8_t address, uint8_t low, uint8_t high)
{
    send_command5(0xFFU, 0xAAU, address, low, high);
}

static void unlock_registers(void)
{
    write_register(JY61_REG_KEY, JY61_KEY_LOW, JY61_KEY_HIGH);
}

static void save_registers(void)
{
    write_register(JY61_REG_SAVE, 0x00U, 0x00U);
}

static void delay_ms(uint32_t ms)
{
    delay_cycles((CPUCLK_FREQ / 1000U) * ms);
}

static void poll_for_ms(uint32_t ms)
{
    for (uint32_t elapsed = 0U; elapsed < ms; elapsed++) {
        delay_ms(1U);
        jy61_poll();
    }
}

static void reset_parser(void)
{
    gRxState = JY61_WAIT_HEAD;
    gFrameType = 0U;
    gPayloadIndex = 0U;
    gAngleValid = false;
    gQuatValid = false;
}
