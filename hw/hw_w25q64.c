#include "hw_w25q64.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

#define W25Q64_CMD_WRITE_ENABLE (0x06U)
#define W25Q64_CMD_READ_STATUS1 (0x05U)
#define W25Q64_CMD_PAGE_PROGRAM (0x02U)
#define W25Q64_CMD_READ_DATA (0x03U)
#define W25Q64_CMD_SECTOR_ERASE (0x20U)
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
static void w25q64_write_enable(void);
static uint8_t w25q64_read_status1(void);
static bool w25q64_wait_ready(void);
static void w25q64_send_address(uint32_t address);

void w25q64_init(void)
{
    W25Q64_CS_HIGH();
    W25Q64_CLK_LOW();
    W25Q64_MOSI_LOW();
}

uint32_t w25q64_read_jedec_id(void)
{
    uint32_t id;

    W25Q64_CS_LOW();
    (void) w25q64_transfer(W25Q64_CMD_JEDEC_ID);
    id = ((uint32_t) w25q64_transfer(0xFFU)) << 16;
    id |= ((uint32_t) w25q64_transfer(0xFFU)) << 8;
    id |= (uint32_t) w25q64_transfer(0xFFU);
    W25Q64_CS_HIGH();

    return id;
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
    uint8_t received = 0U;

    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        W25Q64_CLK_LOW();
        if ((value & mask) != 0U) {
            W25Q64_MOSI_HIGH();
        } else {
            W25Q64_MOSI_LOW();
        }
        delay_cycles(4U);
        W25Q64_CLK_HIGH();
        delay_cycles(4U);
        if ((DL_GPIO_readPins(GPIO_W25Q64_PORT,
                GPIO_W25Q64_W25_MISO_PIN) &
                GPIO_W25Q64_W25_MISO_PIN) != 0U) {
            received |= mask;
        }
    }
    W25Q64_CLK_LOW();

    return received;
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
