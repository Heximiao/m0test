#include "app_image_store.h"
#include "hw/hw_lcd.h"
#include "hw/hw_uart.h"
#include "hw/hw_w25q64.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define IMAGE_MAGIC (0x30474D49UL)
#define IMAGE_FORMAT_RGB565 (1U)
#define IMAGE_SLOT_BASE (0x001000UL)
#define IMAGE_SLOT_SIZE (0x020000UL)
#define IMAGE_MAX_SLOTS (32U)
#define IMAGE_RX_BUFFER_SIZE (256U)
#define IMAGE_DISPLAY_BUFFER_SIZE (256U)

typedef struct {
    uint32_t magic;
    uint16_t width;
    uint16_t height;
    uint8_t format;
    uint8_t reserved[3];
    uint32_t data_size;
    uint32_t crc32;
} ImageHeader;

static bool gReceiving;
static uint8_t gSlot;
static uint16_t gWidth;
static uint16_t gHeight;
static uint32_t gExpectedSize;
static uint32_t gExpectedCrc;
static uint32_t gReceivedSize;
static uint32_t gRunningCrc;
static uint32_t gWriteAddress;
static uint8_t gRxBuffer[IMAGE_RX_BUFFER_SIZE];
static size_t gRxLength;
static uint8_t gDisplayBuffer[IMAGE_DISPLAY_BUFFER_SIZE];

static bool start_image_write(uint8_t slot, uint16_t width, uint16_t height,
    uint32_t size, uint32_t crc32);
static bool erase_range(uint32_t startAddress, uint32_t length);
static void send_page_ack(void);
static void finish_image_write(void);
static uint32_t crc32_update(uint32_t crc, uint8_t byte);
static uint32_t slot_base_address(uint8_t slot);
static void send_flash_id(void);
static bool check_flash_ready(void);
static void show_image_slot(uint8_t slot);

void app_image_store_init(void)
{
    w25q64_init();
}

bool app_image_store_process_command(const char *command)
{
    unsigned int slot;
    unsigned int width;
    unsigned int height;
    unsigned long size;
    unsigned long crc;

    if (strcmp(command, "FLASHID") == 0) {
        send_flash_id();
        return true;
    }

    if (strncmp(command, "IMG_SHOW ", 9U) == 0) {
        unsigned int showSlot;
        if (sscanf(command + 9, "%u", &showSlot) == 1) {
            show_image_slot((uint8_t) showSlot);
        } else {
            uart_debug_write_string("ERR IMG_SHOW\r\n");
        }
        return true;
    }

    if (strncmp(command, "IMG_WRITE ", 10U) != 0) {
        return false;
    }

    if (sscanf(command + 10, "%u %u %u %lu %lx", &slot, &width, &height,
            &size, &crc) != 5) {
        uart_debug_write_string("ERR IMG_ARGS\r\n");
        return true;
    }

    (void) start_image_write((uint8_t) slot, (uint16_t) width,
        (uint16_t) height, (uint32_t) size, (uint32_t) crc);

    return true;
}

bool app_image_store_is_receiving(void)
{
    return gReceiving;
}

void app_image_store_service(void)
{
    uint8_t byte;

    if (!gReceiving) {
        return;
    }

    while (uart_debug_read_byte(&byte)) {
        gRxBuffer[gRxLength++] = byte;
        gRunningCrc = crc32_update(gRunningCrc, byte);
        gReceivedSize++;

        if ((gRxLength == sizeof(gRxBuffer)) ||
            (gReceivedSize == gExpectedSize)) {
            if (!w25q64_write(gWriteAddress, gRxBuffer, gRxLength)) {
                gReceiving = false;
                gRxLength = 0U;
                uart_debug_write_string("ERR IMG_WRITE_FLASH\r\n");
                return;
            }
            gWriteAddress += (uint32_t) gRxLength;
            gRxLength = 0U;
            send_page_ack();
        }

        if (gReceivedSize == gExpectedSize) {
            finish_image_write();
            return;
        }
    }
}

static bool start_image_write(uint8_t slot, uint16_t width, uint16_t height,
    uint32_t size, uint32_t crc32)
{
    uint32_t base = slot_base_address(slot);
    uint32_t totalLength = (uint32_t) sizeof(ImageHeader) + size;

    if (gReceiving || (slot >= IMAGE_MAX_SLOTS) || (width == 0U) ||
        (height == 0U) || (size == 0U) || (totalLength > IMAGE_SLOT_SIZE)) {
        uart_debug_write_string("ERR IMG_CONFIG\r\n");
        return false;
    }

    if (base == 0U) {
        uart_debug_write_string("ERR IMG_ADDRESS\r\n");
        return false;
    }

    if (!check_flash_ready()) {
        return false;
    }

    uart_debug_write_string("OK IMG_ERASE\r\n");
    if (!erase_range(base, totalLength)) {
        uart_debug_write_string("ERR IMG_ERASE\r\n");
        return false;
    }

    gReceiving = true;
    gSlot = slot;
    gWidth = width;
    gHeight = height;
    gExpectedSize = size;
    gExpectedCrc = crc32;
    gReceivedSize = 0U;
    gRunningCrc = 0xFFFFFFFFUL;
    gWriteAddress = base + (uint32_t) sizeof(ImageHeader);
    gRxLength = 0U;
    uart_debug_write_string("OK IMG_READY\r\n");

    return true;
}

static bool erase_range(uint32_t startAddress, uint32_t length)
{
    uint32_t endAddress = startAddress + length;

    for (uint32_t address = startAddress; address < endAddress;
         address += W25Q64_SECTOR_SIZE_BYTES) {
        if (!w25q64_erase_sector(address)) {
            return false;
        }
    }

    return true;
}

static void send_page_ack(void)
{
    char message[40];

    snprintf(message, sizeof(message), "OK IMG_PAGE %lu\r\n",
        (unsigned long) gReceivedSize);
    uart_debug_write_string(message);
}

static void finish_image_write(void)
{
    char message[96];
    uint32_t finalCrc = gRunningCrc ^ 0xFFFFFFFFUL;
    ImageHeader header = {
        IMAGE_MAGIC,
        gWidth,
        gHeight,
        IMAGE_FORMAT_RGB565,
        {0U, 0U, 0U},
        gExpectedSize,
        finalCrc,
    };
    uint32_t base = slot_base_address(gSlot);

    gReceiving = false;
    if (finalCrc != gExpectedCrc) {
        uart_debug_write_string("ERR IMG_CRC\r\n");
        return;
    }

    if (!w25q64_write(base, (const uint8_t *) &header, sizeof(header))) {
        uart_debug_write_string("ERR IMG_HEADER\r\n");
        return;
    }

    snprintf(message, sizeof(message), "OK IMG_DONE SLOT=%u SIZE=%lu CRC=%08lX\r\n",
        gSlot, (unsigned long) gExpectedSize, (unsigned long) finalCrc);
    uart_debug_write_string(message);
}

static uint32_t crc32_update(uint32_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((crc & 1U) != 0U) {
            crc = (crc >> 1U) ^ 0xEDB88320UL;
        } else {
            crc >>= 1U;
        }
    }

    return crc;
}

static uint32_t slot_base_address(uint8_t slot)
{
    return IMAGE_SLOT_BASE + ((uint32_t) slot * IMAGE_SLOT_SIZE);
}

static void send_flash_id(void)
{
    char message[48];
    uint32_t id = w25q64_read_jedec_id();
    snprintf(message, sizeof(message), "FLASH ID=%06lX\r\n",
        (unsigned long) id);
    uart_debug_write_string(message);
}

static bool check_flash_ready(void)
{
    char message[64];
    uint32_t id = w25q64_read_jedec_id();

    if (w25q64_is_valid_jedec_id(id)) {
        return true;
    }

    snprintf(message, sizeof(message), "ERR IMG_FLASH_ID ID=%06lX\r\n",
        (unsigned long) id);
    uart_debug_write_string(message);
    return false;
}

static void show_image_slot(uint8_t slot)
{
    ImageHeader header;
    uint32_t base = slot_base_address(slot);
    uint32_t dataAddress;
    uint32_t remaining;

    if (slot >= IMAGE_MAX_SLOTS) {
        uart_debug_write_string("ERR IMG_SLOT\r\n");
        return;
    }

    if (!w25q64_read(base, (uint8_t *) &header, sizeof(header))) {
        uart_debug_write_string("ERR IMG_READ_HEADER\r\n");
        return;
    }

    if ((header.magic != IMAGE_MAGIC) ||
        (header.format != IMAGE_FORMAT_RGB565) ||
        (header.width == 0U) ||
        (header.height == 0U) ||
        (header.data_size !=
            ((uint32_t) header.width * (uint32_t) header.height * 2UL))) {
        uart_debug_write_string("ERR IMG_HEADER_INVALID\r\n");
        return;
    }

    LCD_Address_Set(0U, 0U, header.width - 1U, header.height - 1U);
    dataAddress = base + (uint32_t) sizeof(ImageHeader);
    remaining = header.data_size;
    while (remaining > 0U) {
        size_t chunk = sizeof(gDisplayBuffer);
        if (chunk > remaining) {
            chunk = remaining;
        }
        if (!w25q64_read(dataAddress, gDisplayBuffer, chunk)) {
            uart_debug_write_string("ERR IMG_READ_DATA\r\n");
            return;
        }
        for (size_t i = 0U; i < chunk; i++) {
            LCD_WR_DATA8(gDisplayBuffer[i]);
        }
        dataAddress += (uint32_t) chunk;
        remaining -= (uint32_t) chunk;
    }

    uart_debug_write_string("OK IMG_SHOW\r\n");
}
