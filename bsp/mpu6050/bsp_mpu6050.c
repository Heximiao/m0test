#include "bsp_mpu6050.h"

#include "inv_mpu.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stddef.h>

#define MPU6050_SMPLRT_DIV_REG (0x19U)
#define MPU6050_CONFIG_REG (0x1AU)
#define MPU6050_GYRO_CONFIG_REG (0x1BU)
#define MPU6050_ACCEL_CONFIG_REG (0x1CU)
#define MPU6050_FIFO_EN_REG (0x23U)
#define MPU6050_INT_PIN_CFG_REG (0x37U)
#define MPU6050_INT_ENABLE_REG (0x38U)
#define MPU6050_USER_CTRL_REG (0x6AU)
#define MPU6050_PWR_MGMT_1_REG (0x6BU)
#define MPU6050_PWR_MGMT_2_REG (0x6CU)
#define MPU6050_WHO_AM_I_REG (0x75U)

static void iic_start(void);
static void iic_stop(void);
static void iic_send_ack(bool nack);
static bool iic_wait_ack(void);
static void iic_send_byte(uint8_t data);
static uint8_t iic_read_byte(void);
static void sda_output(void);
static void sda_input(void);
static void sda_write(bool high);
static void scl_write(bool high);
static bool sda_read(void);
static char write_byte(uint8_t reg, uint8_t value);
static uint8_t set_gyro_fsr(uint8_t fsr);
static uint8_t set_accel_fsr(uint8_t fsr);
static uint8_t set_lpf(uint16_t lpf);
static uint8_t set_sample_rate(uint16_t rate);

char MPU6050_WriteReg(uint8_t addr, uint8_t regaddr, uint8_t num,
    uint8_t *regdata)
{
    iic_start();
    iic_send_byte((uint8_t) ((addr << 1U) | 0U));
    if (!iic_wait_ack()) {
        iic_stop();
        return 1;
    }
    iic_send_byte(regaddr);
    if (!iic_wait_ack()) {
        iic_stop();
        return 2;
    }

    for (uint8_t i = 0U; i < num; i++) {
        iic_send_byte(regdata[i]);
        if (!iic_wait_ack()) {
            iic_stop();
            return (char) (3U + i);
        }
    }

    iic_stop();
    return 0;
}

char MPU6050_ReadData(uint8_t addr, uint8_t regaddr, uint8_t num,
    uint8_t *read)
{
    if ((num == 0U) || (read == NULL)) {
        return 1;
    }

    iic_start();
    iic_send_byte((uint8_t) ((addr << 1U) | 0U));
    if (!iic_wait_ack()) {
        iic_stop();
        return 2;
    }
    iic_send_byte(regaddr);
    if (!iic_wait_ack()) {
        iic_stop();
        return 3;
    }

    iic_start();
    iic_send_byte((uint8_t) ((addr << 1U) | 1U));
    if (!iic_wait_ack()) {
        iic_stop();
        return 4;
    }

    for (uint8_t i = 0U; i < num; i++) {
        read[i] = iic_read_byte();
        iic_send_ack(i == (uint8_t) (num - 1U));
    }
    iic_stop();
    return 0;
}

bool mpu6050_init(void)
{
    sda_output();
    mpu6050_release_bus();
    mpu6050_delay_ms(10U);

    if (write_byte(MPU6050_PWR_MGMT_1_REG, 0x80U) != 0) {
        return false;
    }
    mpu6050_delay_ms(100U);

    if (write_byte(MPU6050_PWR_MGMT_1_REG, 0x00U) != 0) {
        return false;
    }
    if ((set_gyro_fsr(3U) != 0U) || (set_accel_fsr(0U) != 0U) ||
        (set_sample_rate(50U) != 0U)) {
        return false;
    }

    if ((write_byte(MPU6050_INT_ENABLE_REG, 0x00U) != 0) ||
        (write_byte(MPU6050_USER_CTRL_REG, 0x00U) != 0) ||
        (write_byte(MPU6050_FIFO_EN_REG, 0x00U) != 0) ||
        (write_byte(MPU6050_INT_PIN_CFG_REG, 0x80U) != 0)) {
        return false;
    }

    uint8_t whoAmI = mpu6050_read_id();
    if ((whoAmI != 0x68U) && (whoAmI != 0x70U) && (whoAmI != 0x71U)) {
        return false;
    }

    return (write_byte(MPU6050_PWR_MGMT_1_REG, 0x01U) == 0) &&
        (write_byte(MPU6050_PWR_MGMT_2_REG, 0x00U) == 0) &&
        (set_sample_rate(50U) == 0U);
}

bool mpu6050_dmp_init(void)
{
    return mpu_dmp_init() == 0U;
}

bool mpu6050_read_angles(Mpu6050Angles *angles)
{
    if (angles == NULL) {
        return false;
    }

    return mpu_dmp_get_data(&angles->pitch, &angles->roll, &angles->yaw) == 0U;
}

bool mpu6050_read_accel_angles(Mpu6050Angles *angles)
{
    uint8_t raw[6];
    float ax;
    float ay;
    float az;

    if (angles == NULL) {
        return false;
    }
    if (MPU6050_ReadData(MPU6050_I2C_ADDR, 0x3BU, sizeof(raw), raw) != 0) {
        return false;
    }

    ax = (float) (int16_t) (((uint16_t) raw[0] << 8U) | raw[1]);
    ay = (float) (int16_t) (((uint16_t) raw[2] << 8U) | raw[3]);
    az = (float) (int16_t) (((uint16_t) raw[4] << 8U) | raw[5]);
    angles->pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 57.29578f;
    angles->roll = atan2f(ay, az) * 57.29578f;
    angles->yaw = 0.0f;
    return true;
}

uint8_t mpu6050_read_id(void)
{
    uint8_t id = 0U;
    if (MPU6050_ReadData(MPU6050_I2C_ADDR, MPU6050_WHO_AM_I_REG, 1U, &id) !=
        0) {
        return 0U;
    }
    return id;
}

void mpu6050_release_bus(void)
{
    sda_output();
    sda_write(true);
    scl_write(true);
    mpu6050_delay_us(10U);
}

bool mpu6050_scl_is_high(void)
{
    return (DL_GPIO_readPins(GPIO_MPU6050_I2C_PORT,
        GPIO_MPU6050_I2C_SCL_PIN) & GPIO_MPU6050_I2C_SCL_PIN) != 0U;
}

bool mpu6050_sda_is_high(void)
{
    return sda_read();
}

void mpu6050_delay_us(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);
}

void mpu6050_delay_ms(uint32_t ms)
{
    delay_cycles((CPUCLK_FREQ / 1000U) * ms);
}

static void iic_start(void)
{
    sda_output();
    scl_write(true);
    sda_write(true);
    mpu6050_delay_us(5U);
    sda_write(false);
    mpu6050_delay_us(5U);
    scl_write(false);
}

static void iic_stop(void)
{
    sda_output();
    scl_write(false);
    sda_write(false);
    scl_write(true);
    mpu6050_delay_us(5U);
    sda_write(true);
    mpu6050_delay_us(5U);
}

static void iic_send_ack(bool nack)
{
    sda_output();
    scl_write(false);
    sda_write(nack);
    mpu6050_delay_us(5U);
    scl_write(true);
    mpu6050_delay_us(5U);
    scl_write(false);
    sda_write(true);
}

static bool iic_wait_ack(void)
{
    uint8_t timeout = 10U;

    scl_write(false);
    sda_write(true);
    sda_input();
    scl_write(true);

    while (sda_read() && (timeout > 0U)) {
        timeout--;
        mpu6050_delay_us(5U);
    }

    scl_write(false);
    sda_output();

    if (timeout == 0U) {
        iic_stop();
        return false;
    }
    return true;
}

static void iic_send_byte(uint8_t data)
{
    sda_output();
    scl_write(false);

    for (uint8_t i = 0U; i < 8U; i++) {
        sda_write((data & 0x80U) != 0U);
        mpu6050_delay_us(1U);
        scl_write(true);
        mpu6050_delay_us(5U);
        scl_write(false);
        mpu6050_delay_us(5U);
        data <<= 1U;
    }
}

static uint8_t iic_read_byte(void)
{
    uint8_t data = 0U;

    sda_input();
    for (uint8_t i = 0U; i < 8U; i++) {
        scl_write(false);
        mpu6050_delay_us(5U);
        scl_write(true);
        mpu6050_delay_us(5U);
        data <<= 1U;
        if (sda_read()) {
            data |= 1U;
        }
        mpu6050_delay_us(5U);
    }
    scl_write(false);
    return data;
}

static void sda_output(void)
{
    DL_GPIO_initDigitalOutput(GPIO_MPU6050_I2C_SDA_IOMUX);
    DL_GPIO_setPins(GPIO_MPU6050_I2C_PORT, GPIO_MPU6050_I2C_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_MPU6050_I2C_PORT, GPIO_MPU6050_I2C_SDA_PIN);
}

static void sda_input(void)
{
    DL_GPIO_initDigitalInput(GPIO_MPU6050_I2C_SDA_IOMUX);
}

static void sda_write(bool high)
{
    if (high) {
        DL_GPIO_setPins(GPIO_MPU6050_I2C_PORT, GPIO_MPU6050_I2C_SDA_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MPU6050_I2C_PORT, GPIO_MPU6050_I2C_SDA_PIN);
    }
}

static void scl_write(bool high)
{
    if (high) {
        DL_GPIO_setPins(GPIO_MPU6050_I2C_PORT, GPIO_MPU6050_I2C_SCL_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MPU6050_I2C_PORT, GPIO_MPU6050_I2C_SCL_PIN);
    }
}

static bool sda_read(void)
{
    return (DL_GPIO_readPins(GPIO_MPU6050_I2C_PORT,
        GPIO_MPU6050_I2C_SDA_PIN) & GPIO_MPU6050_I2C_SDA_PIN) != 0U;
}

static char write_byte(uint8_t reg, uint8_t value)
{
    return MPU6050_WriteReg(MPU6050_I2C_ADDR, reg, 1U, &value);
}

static uint8_t set_gyro_fsr(uint8_t fsr)
{
    return (uint8_t) write_byte(MPU6050_GYRO_CONFIG_REG,
        (uint8_t) (fsr << 3U));
}

static uint8_t set_accel_fsr(uint8_t fsr)
{
    return (uint8_t) write_byte(MPU6050_ACCEL_CONFIG_REG,
        (uint8_t) (fsr << 3U));
}

static uint8_t set_lpf(uint16_t lpf)
{
    uint8_t data;

    if (lpf >= 188U) {
        data = 1U;
    } else if (lpf >= 98U) {
        data = 2U;
    } else if (lpf >= 42U) {
        data = 3U;
    } else if (lpf >= 20U) {
        data = 4U;
    } else if (lpf >= 10U) {
        data = 5U;
    } else {
        data = 6U;
    }

    return (uint8_t) write_byte(MPU6050_CONFIG_REG, data);
}

static uint8_t set_sample_rate(uint16_t rate)
{
    uint8_t data;

    if (rate > 1000U) {
        rate = 1000U;
    }
    if (rate < 4U) {
        rate = 4U;
    }

    data = (uint8_t) ((1000U / rate) - 1U);
    if (write_byte(MPU6050_SMPLRT_DIV_REG, data) != 0) {
        return 1U;
    }
    return set_lpf((uint16_t) (rate / 2U));
}
