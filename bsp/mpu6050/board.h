#ifndef MPU6050_BOARD_COMPAT_H
#define MPU6050_BOARD_COMPAT_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

void mpu6050_delay_us(uint32_t us);
void mpu6050_delay_ms(uint32_t ms);

#define delay_us mpu6050_delay_us
#define delay_ms mpu6050_delay_ms
#define delay_1us mpu6050_delay_us
#define delay_1ms mpu6050_delay_ms

#endif
