#include "hw_w25q64.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

#define W25Q64_CMD_WRITE_ENABLE (0x06U)
#define W25Q64_CMD_READ_STATUS1 (0x05U)
#define W25Q64_CMD_PAGE_PROGRAM (0x02U)
#define W25Q64_CMD_READ_DATA (0x03U)
#define W25Q64_CMD_SECTOR_ERASE (0x20U)
#define W25Q64_CMD_RESET_ENABLE (0x66U)
#define W25Q64_CMD_RESET_DEVICE (0x99U)
#define W25Q64_CMD_RELEASE_POWER_DOWN (0xABU)
#define W25Q64_CMD_JEDEC_ID (0x9FU)
#define W25Q64_STATUS_BUSY (0x01U)
#define W25Q64_BUSY_TIMEOUT_LOOPS (300000UL)

#define W25Q64_CS_LOW() \
    DL_GPIO_clearPins(GPIO_W25Q64_PORT, GPIO_W25Q64_W25_CS_PIN)
#define W25Q64_CS_HIGH() \
    DL_GPIO_setPins(GPIO_W25Q64_PORT, GPIO_W25Q64_W25_CS_PIN)
#define W25Q64_CLK_LOW() \
    DL_GPIO_clearPins(GPIO_W25Q64_PORT, GPIO_W25Q64_W25_SCLK_PIN)
#define W25Q64_CLK_HIGH() \
    DL_GPIO_setPins(GPIO_W25Q64_PORT, GPIO_W25Q64_W25_SCLK_PIN)
#define W25Q64_MOSI_LOW() \
    DL_GPIO_clearPins(GPIO_W25Q64_PORT, GPIO_W25Q64_W25_MOSI_PIN)
#define W25Q64_MOSI_HIGH() \
    DL_GPIO_setPins(GPIO_W25Q64_PORT, GPIO_W25Q64_W25_MOSI_PIN)

static uint8_t w25q64_transfer(uint8_t value);
static uint8_t w25q64_transfer_pins(uint8_t value, uint32_t mosiPin,
    uint32_t misoPin);
static void w25q64_configure_pins(void);
static void w25q64_configure_probe_pins(bool swapped);
static uint32_t w25q64_read_jedec_id_probe(bool swapped);
static void w25q64_command(uint8_t command);
static void w25q64_write_enable(void);
static uint8_t w25q64_read_status1(void);
static bool w25q64_wait_ready(void);
static void w25q64_send_address(uint32_t address);

void w25q64_init(void)
{
    w25q64_configure_pins();
    delay_cycles(8000U);

    w25q64_command(W25Q64_CMD_RELEASE_POWER_DOWN);
    delay_cycles(8000U);
    w25q64_command(W25Q64_CMD_RESET_ENABLE);
    w25q64_command(W25Q64_CMD_RESET_DEVICE);
    delay_cycles(8000U);
}

static void w25q64_configure_pins(void)
{
    DL_GPIO_initDigitalOutput(GPIO_W25Q64_W25_CS_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_W25Q64_W25_SCLK_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_W25Q64_W25_MOSI_IOMUX);
    DL_GPIO_initDigitalInputFeatures(GPIO_W25Q64_W25_MISO_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableOutput(GPIO_W25Q64_PORT,
        GPIO_W25Q64_W25_CS_PIN | GPIO_W25Q64_W25_SCLK_PIN |
        GPIO_W25Q64_W25_MOSI_PIN);

    W25Q64_CS_HIGH();
    W25Q64_CLK_LOW();
    W25Q64_MOSI_LOW();
}

uint32_t w25q64_read_jedec_id(void)
{
    uint32_t id;

    w25q64_configure_pins();
    W25Q64_CS_LOW();
    (void) w25q64_transfer(W25Q64_CMD_JEDEC_ID);
    id = ((uint32_t) w25q64_transfer(0xFFU)) << 16;
    id |= ((uint32_t) w25q64_transfer(0xFFU)) << 8;
    id |= (uint32_t) w25q64_transfer(0xFFU);
    W25Q64_CS_HIGH();

    return id;
}

uint32_t w25q64_read_jedec_id_atomic(void)
{
    uint32_t id;

    __disable_irq();
    id = w25q64_read_jedec_id();
    __enable_irq();

    return id;
}

uint32_t w25q64_read_gpio_levels(void)
{
    uint32_t pins = GPIO_W25Q64_PORT->DIN31_0;
    uint32_t output = GPIO_W25Q64_PORT->DOUT31_0;
    uint32_t enabled = GPIO_W25Q64_PORT->DOE31_0;
    uint32_t levels = 0UL;

    if ((output & GPIO_W25Q64_W25_CS_PIN) != 0U) {
        levels |= 0x8UL;
    }
    if ((output & GPIO_W25Q64_W25_SCLK_PIN) != 0U) {
        levels |= 0x4UL;
    }
    if ((output & GPIO_W25Q64_W25_MOSI_PIN) != 0U) {
        levels |= 0x2UL;
    }
    if ((pins & GPIO_W25Q64_W25_MISO_PIN) != 0U) {
        levels |= 0x1UL;
    }
    if ((enabled & GPIO_W25Q64_W25_CS_PIN) != 0U) {
        levels |= 0x80UL;
    }
    if ((enabled & GPIO_W25Q64_W25_SCLK_PIN) != 0U) {
        levels |= 0x40UL;
    }
    if ((enabled & GPIO_W25Q64_W25_MOSI_PIN) != 0U) {
        levels |= 0x20UL;
    }

    return levels;
}

void w25q64_probe_jedec_ids(uint32_t *normalId, uint32_t *swappedId)
{
    uint32_t normal;
    uint32_t swapped;

    __disable_irq();
    normal = w25q64_read_jedec_id_probe(false);
    swapped = w25q64_read_jedec_id_probe(true);
    w25q64_configure_pins();
    __enable_irq();

    if (normalId != NULL) {
        *normalId = normal;
    }
    if (swappedId != NULL) {
        *swappedId = swapped;
    }
}

bool w25q64_is_valid_jedec_id(uint32_t id)
{
    uint8_t memoryType = (uint8_t) ((id >> 8) & 0xFFU);
    uint8_t capacity = (uint8_t) (id & 0xFFU);

    if ((id == 0UL) || (id == 0xFFFFFFUL)) {
        return false;
    }

    return (memoryType == 0x40U) && (capacity == 0x17U);
}

bool w25q64_erase_sector(uint32_t address)
{
    w25q64_configure_pins();
    if (address >= W25Q64_TOTAL_SIZE_BYTES) {
        return false;
    }

    address &= ~(W25Q64_SECTOR_SIZE_BYTES - 1UL);
    w25q64_write_enable();
    W25Q64_CS_LOW();
    (void) w25q64_transfer(W25Q64_CMD_SECTOR_ERASE);
    w25q64_send_address(address);
    W25Q64_CS_HIGH();

    return w25q64_wait_ready();
}

bool w25q64_read(uint32_t address, uint8_t *buffer, size_t length)
{
    w25q64_configure_pins();
    if ((buffer == NULL) ||
        ((address + (uint32_t) length) > W25Q64_TOTAL_SIZE_BYTES)) {
        return false;
    }

    W25Q64_CS_LOW();
    (void) w25q64_transfer(W25Q64_CMD_READ_DATA);
    w25q64_send_address(address);
    for (size_t i = 0U; i < length; i++) {
        buffer[i] = w25q64_transfer(0xFFU);
    }
    W25Q64_CS_HIGH();

    return true;
}

bool w25q64_write(uint32_t address, const uint8_t *buffer, size_t length)
{
    w25q64_configure_pins();
    if ((buffer == NULL) ||
        ((address + (uint32_t) length) > W25Q64_TOTAL_SIZE_BYTES)) {
        return false;
    }

    while (length > 0U) {
        uint32_t pageOffset = address % W25Q64_PAGE_SIZE_BYTES;
        size_t chunk = W25Q64_PAGE_SIZE_BYTES - pageOffset;
        if (chunk > length) {
            chunk = length;
        }

        w25q64_write_enable();
        W25Q64_CS_LOW();
        (void) w25q64_transfer(W25Q64_CMD_PAGE_PROGRAM);
        w25q64_send_address(address);
        for (size_t i = 0U; i < chunk; i++) {
            (void) w25q64_transfer(buffer[i]);
        }
        W25Q64_CS_HIGH();
        if (!w25q64_wait_ready()) {
            return false;
        }

        address += (uint32_t) chunk;
        buffer += chunk;
        length -= chunk;
    }

    return true;
}

static uint8_t w25q64_transfer(uint8_t value)
{
    return w25q64_transfer_pins(value, GPIO_W25Q64_W25_MOSI_PIN,
        GPIO_W25Q64_W25_MISO_PIN);
}

static uint8_t w25q64_transfer_pins(uint8_t value, uint32_t mosiPin,
    uint32_t misoPin)
{
    uint8_t received = 0U;

    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        W25Q64_CLK_LOW();
        if ((value & mask) != 0U) {
            DL_GPIO_setPins(GPIO_W25Q64_PORT, mosiPin);
        } else {
            DL_GPIO_clearPins(GPIO_W25Q64_PORT, mosiPin);
        }
        delay_cycles(4U);
        W25Q64_CLK_HIGH();
        delay_cycles(4U);
        if ((DL_GPIO_readPins(GPIO_W25Q64_PORT, misoPin) & misoPin) != 0U) {
            received |= mask;
        }
    }
    W25Q64_CLK_LOW();

    return received;
}

static void w25q64_configure_probe_pins(bool swapped)
{
    uint32_t mosiPin = swapped ? GPIO_W25Q64_W25_MISO_PIN :
        GPIO_W25Q64_W25_MOSI_PIN;
    uint32_t misoPin = swapped ? GPIO_W25Q64_W25_MOSI_PIN :
        GPIO_W25Q64_W25_MISO_PIN;
    uint32_t mosiIomux = swapped ? GPIO_W25Q64_W25_MISO_IOMUX :
        GPIO_W25Q64_W25_MOSI_IOMUX;
    uint32_t misoIomux = swapped ? GPIO_W25Q64_W25_MOSI_IOMUX :
        GPIO_W25Q64_W25_MISO_IOMUX;

    DL_GPIO_initDigitalOutput(GPIO_W25Q64_W25_CS_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_W25Q64_W25_SCLK_IOMUX);
    DL_GPIO_initDigitalOutput(mosiIomux);
    DL_GPIO_initDigitalInputFeatures(misoIomux, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(GPIO_W25Q64_PORT, misoPin);
    DL_GPIO_enableOutput(GPIO_W25Q64_PORT,
        GPIO_W25Q64_W25_CS_PIN | GPIO_W25Q64_W25_SCLK_PIN | mosiPin);
    W25Q64_CS_HIGH();
    W25Q64_CLK_LOW();
    DL_GPIO_clearPins(GPIO_W25Q64_PORT, mosiPin);
}

static uint32_t w25q64_read_jedec_id_probe(bool swapped)
{
    uint32_t id;
    uint32_t mosiPin = swapped ? GPIO_W25Q64_W25_MISO_PIN :
        GPIO_W25Q64_W25_MOSI_PIN;
    uint32_t misoPin = swapped ? GPIO_W25Q64_W25_MOSI_PIN :
        GPIO_W25Q64_W25_MISO_PIN;

    w25q64_configure_probe_pins(swapped);
    W25Q64_CS_LOW();
    (void) w25q64_transfer_pins(W25Q64_CMD_JEDEC_ID, mosiPin, misoPin);
    id = ((uint32_t) w25q64_transfer_pins(0xFFU, mosiPin, misoPin)) << 16;
    id |= ((uint32_t) w25q64_transfer_pins(0xFFU, mosiPin, misoPin)) << 8;
    id |= (uint32_t) w25q64_transfer_pins(0xFFU, mosiPin, misoPin);
    W25Q64_CS_HIGH();

    return id;
}

static void w25q64_command(uint8_t command)
{
    W25Q64_CS_LOW();
    (void) w25q64_transfer(command);
    W25Q64_CS_HIGH();
}

static void w25q64_write_enable(void)
{
    W25Q64_CS_LOW();
    (void) w25q64_transfer(W25Q64_CMD_WRITE_ENABLE);
    W25Q64_CS_HIGH();
}

static uint8_t w25q64_read_status1(void)
{
    uint8_t status;

    W25Q64_CS_LOW();
    (void) w25q64_transfer(W25Q64_CMD_READ_STATUS1);
    status = w25q64_transfer(0xFFU);
    W25Q64_CS_HIGH();

    return status;
}

static bool w25q64_wait_ready(void)
{
    for (uint32_t i = 0U; i < W25Q64_BUSY_TIMEOUT_LOOPS; i++) {
        uint8_t status = w25q64_read_status1();
        if (status == 0xFFU) {
            return false;
        }
        if ((status & W25Q64_STATUS_BUSY) == 0U) {
            return true;
        }
    }

    return false;
}

static void w25q64_send_address(uint32_t address)
{
    (void) w25q64_transfer((uint8_t) ((address >> 16) & 0xFFU));
    (void) w25q64_transfer((uint8_t) ((address >> 8) & 0xFFU));
    (void) w25q64_transfer((uint8_t) (address & 0xFFU));
}
