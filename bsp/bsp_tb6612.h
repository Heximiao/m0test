#ifndef BSP_TB6612_H
#define BSP_TB6612_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define PWM_PERIOD_COUNTS (1600U)
#define TB6612_SPEED_MAX (1000U)

#define AIN1_OUT(x) \
    ((x) ? DL_GPIO_setPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN1_PIN) : \
           DL_GPIO_clearPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN1_PIN))
#define AIN2_OUT(x) \
    ((x) ? DL_GPIO_setPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN2_PIN) : \
           DL_GPIO_clearPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_AIN2_PIN))
#define BIN1_OUT(x) \
    ((x) ? DL_GPIO_setPins(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_BIN1_PIN) : \
           DL_GPIO_clearPins(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_BIN1_PIN))
#define BIN2_OUT(x) \
    ((x) ? DL_GPIO_setPins(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_BIN2_PIN) : \
           DL_GPIO_clearPins(GPIO_MOTOR_B_PORT, GPIO_MOTOR_B_BIN2_PIN))
#define STBY_OUT(x) \
    ((x) ? DL_GPIO_setPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_STBY_PIN) : \
           DL_GPIO_clearPins(GPIO_MOTOR_A_PORT, GPIO_MOTOR_A_STBY_PIN))

void TB6612_Init(void);
void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

#endif
