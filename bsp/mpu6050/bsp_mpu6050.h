#ifndef BSP_MPU6050_H
#define BSP_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_I2C_ADDR (0x68U)

typedef struct {
    float pitch;
    float roll;
    float yaw;
} Mpu6050Angles;

char MPU6050_WriteReg(uint8_t addr, uint8_t regaddr, uint8_t num,
    uint8_t *regdata);
char MPU6050_ReadData(uint8_t addr, uint8_t regaddr, uint8_t num,
    uint8_t *read);

bool mpu6050_init(void);
bool mpu6050_dmp_init(void);
bool mpu6050_read_angles(Mpu6050Angles *angles);
bool mpu6050_read_accel_angles(Mpu6050Angles *angles);
uint8_t mpu6050_read_id(void);
void mpu6050_release_bus(void);
bool mpu6050_scl_is_high(void);
bool mpu6050_sda_is_high(void);

void mpu6050_delay_us(uint32_t us);
void mpu6050_delay_ms(uint32_t ms);

#endif
