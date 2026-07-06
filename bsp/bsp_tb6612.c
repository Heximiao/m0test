#include "bsp_tb6612.h"

static uint32_t speedToCompare(uint32_t speed)
{
    if (speed > TB6612_SPEED_MAX) {
        speed = TB6612_SPEED_MAX;
    }

    uint32_t compare =
        ((TB6612_SPEED_MAX - speed) * PWM_PERIOD_COUNTS) / TB6612_SPEED_MAX;
    if (compare > PWM_PERIOD_COUNTS) {
        compare = PWM_PERIOD_COUNTS;
    }
    if (compare == 0U) {
        return 0U;
    }

    return compare - 1U;
}

void TB6612_Init(void)
{
    TB6612_Motor_Stop();
    DL_TimerA_startCounter(PWM_LEFT_INST);
    DL_TimerA_startCounter(PWM_RIGHT_INST);
}

void TB6612_Motor_Stop(void)
{
    DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, speedToCompare(0U),
        GPIO_PWM_LEFT_C0_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, speedToCompare(0U),
        GPIO_PWM_RIGHT_C0_IDX);

    AIN1_OUT(1);
    AIN2_OUT(1);
    BIN1_OUT(1);
    BIN2_OUT(1);
    STBY_OUT(0);
}

void AO_Control(uint8_t dir, uint32_t speed)
{
    if (speed > TB6612_SPEED_MAX) {
        speed = TB6612_SPEED_MAX;
    }

    STBY_OUT(1);

    if (dir == 1U) {
        AIN1_OUT(0);
        AIN2_OUT(1);
    } else {
        AIN1_OUT(1);
        AIN2_OUT(0);
    }

    DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, speedToCompare(speed),
        GPIO_PWM_LEFT_C0_IDX);
}

void BO_Control(uint8_t dir, uint32_t speed)
{
    if (speed > TB6612_SPEED_MAX) {
        speed = TB6612_SPEED_MAX;
    }

    STBY_OUT(1);

    if (dir == 1U) {
        BIN1_OUT(0);
        BIN2_OUT(1);
    } else {
        BIN1_OUT(1);
        BIN2_OUT(0);
    }

    DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, speedToCompare(speed),
        GPIO_PWM_RIGHT_C0_IDX);
}
