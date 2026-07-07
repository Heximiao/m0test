#ifndef BSP_JY61_H
#define BSP_JY61_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pitch;
    float roll;
    float yaw;
} Jy61Angles;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Jy61Quaternion;

typedef struct {
    uint32_t byteCount;
    uint32_t angleFrameCount;
    uint32_t quatFrameCount;
    uint32_t checksumFailCount;
    uint32_t irqCount;
    uint32_t pollCount;
    uint32_t rxRawStatus;
    uint32_t rxEnabledStatus;
    uint32_t errorStatus;
    uint8_t lastFrameType;
    uint8_t lastFrame[11];
    bool rxPinHigh;
    bool rxFifoEmpty;
    bool rxFifoFull;
    bool angleValid;
    bool quatValid;
} Jy61Debug;

bool jy61_init(void);
bool jy61_configure_output(void);
bool jy61_read_angles(Jy61Angles *angles);
bool jy61_read_quaternion(Jy61Quaternion *quat);
bool jy61_zero_yaw(void);
bool jy61_read_debug(Jy61Debug *debug);
bool jy61_set_baudrate(uint32_t baudrate);
bool jy61_probe_baudrate(uint32_t baudrate, uint32_t listenMs);
uint32_t jy61_get_baudrate(void);
void jy61_poll(void);
void jy61_handle_irq(void);

#endif
