#include "hw_encoder.h"

/*
 * If one encoder count decreases while the car moves forward, flip only that
 * motor's polarity between 1 and -1.
 */
#define LEFT_ENCODER_POLARITY  (-1)
#define RIGHT_ENCODER_POLARITY (1)

static volatile int32_t gLeftEncoderCount;
static volatile int32_t gRightEncoderCount;
static void update_left_from_phase_a(void);
static void update_left_from_phase_b(void);
static void update_right_from_phase_a(void);
static void update_right_from_phase_b(void);

void encoder_init(void)
{
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_LEFT_LEFT_C0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_LEFT_LEFT_C1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_RIGHT_RIGHT_C0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_RIGHT_RIGHT_C1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearInterruptStatus(GPIOB,
        GPIO_ENCODER_LEFT_LEFT_C0_PIN | GPIO_ENCODER_LEFT_LEFT_C1_PIN |
        GPIO_ENCODER_RIGHT_RIGHT_C0_PIN | GPIO_ENCODER_RIGHT_RIGHT_C1_PIN);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void encoder_handle_gpio_irq(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOB,
        GPIO_ENCODER_LEFT_LEFT_C0_PIN | GPIO_ENCODER_LEFT_LEFT_C1_PIN |
        GPIO_ENCODER_RIGHT_RIGHT_C0_PIN | GPIO_ENCODER_RIGHT_RIGHT_C1_PIN);

    if ((pending & GPIO_ENCODER_LEFT_LEFT_C0_PIN) != 0U) {
        update_left_from_phase_a();
    }
    if ((pending & GPIO_ENCODER_LEFT_LEFT_C1_PIN) != 0U) {
        update_left_from_phase_b();
    }
    if ((pending & GPIO_ENCODER_RIGHT_RIGHT_C0_PIN) != 0U) {
        update_right_from_phase_a();
    }
    if ((pending & GPIO_ENCODER_RIGHT_RIGHT_C1_PIN) != 0U) {
        update_right_from_phase_b();
    }

    DL_GPIO_clearInterruptStatus(GPIOB,
        GPIO_ENCODER_LEFT_LEFT_C0_PIN | GPIO_ENCODER_LEFT_LEFT_C1_PIN |
        GPIO_ENCODER_RIGHT_RIGHT_C0_PIN | GPIO_ENCODER_RIGHT_RIGHT_C1_PIN);
}

EncoderCounts encoder_get_counts(void)
{
    EncoderCounts counts;

    __disable_irq();
    counts.left_count = gLeftEncoderCount;
    counts.right_count = gRightEncoderCount;
    __enable_irq();

    return counts;
}

void encoder_reset(void)
{
    __disable_irq();
    gLeftEncoderCount = 0;
    gRightEncoderCount = 0;
    __enable_irq();
}

static void update_left_from_phase_a(void)
{
    if (!DL_GPIO_readPins(GPIO_ENCODER_LEFT_PORT,
            GPIO_ENCODER_LEFT_LEFT_C1_PIN)) {
        gLeftEncoderCount -= LEFT_ENCODER_POLARITY;
    } else {
        gLeftEncoderCount += LEFT_ENCODER_POLARITY;
    }
}

static void update_left_from_phase_b(void)
{
    if (!DL_GPIO_readPins(GPIO_ENCODER_LEFT_PORT,
            GPIO_ENCODER_LEFT_LEFT_C0_PIN)) {
        gLeftEncoderCount += LEFT_ENCODER_POLARITY;
    } else {
        gLeftEncoderCount -= LEFT_ENCODER_POLARITY;
    }
}

static void update_right_from_phase_a(void)
{
    if (!DL_GPIO_readPins(GPIO_ENCODER_RIGHT_PORT,
            GPIO_ENCODER_RIGHT_RIGHT_C1_PIN)) {
        gRightEncoderCount -= RIGHT_ENCODER_POLARITY;
    } else {
        gRightEncoderCount += RIGHT_ENCODER_POLARITY;
    }
}

static void update_right_from_phase_b(void)
{
    if (!DL_GPIO_readPins(GPIO_ENCODER_RIGHT_PORT,
            GPIO_ENCODER_RIGHT_RIGHT_C0_PIN)) {
        gRightEncoderCount += RIGHT_ENCODER_POLARITY;
    } else {
        gRightEncoderCount -= RIGHT_ENCODER_POLARITY;
    }
}
