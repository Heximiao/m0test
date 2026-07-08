#include "app_image_store.h"
#include "hw/hw_lcd.h"
#include "hw/hw_uart.h"
#include "hw/hw_w25q64.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define IMAGE_MAGIC (0x30474D49UL)
#define IMAGE_FORMAT_RGB565 (1U)
#define IMAGE_INDEX_MAGIC (0x5844494DUL)
#define IMAGE_INDEX_VERSION (1U)
#define IMAGE_INDEX_SIZE (0x010000UL)
#define IMAGE_INDEX_BASE (W25Q64_TOTAL_SIZE_BYTES - IMAGE_INDEX_SIZE)
#define IMAGE_DATA_BASE (0x001000UL)
#define IMAGE_INDEX_SECTOR_COUNT (IMAGE_INDEX_SIZE / W25Q64_SECTOR_SIZE_BYTES)
#define IMAGE_MAX_FILES (96U)
#define IMAGE_MAX_NAME_LENGTH (15U)
#define IMAGE_RX_BUFFER_SIZE (256U)
#define IMAGE_DISPLAY_BUFFER_SIZE (256U)
#define IMAGE_STATE_EMPTY (0xFFU)
#define IMAGE_STATE_ACTIVE (0xA5U)
#define IMAGE_STATE_DELETED (0x00U)
#define IMAGE_LEGACY_SLOT_BASE (0x001000UL)
#define IMAGE_LEGACY_SLOT_SIZE (0x020000UL)
#define IMAGE_LEGACY_MAX_SLOTS (32U)

typedef struct {
    uint32_t magic;
    uint16_t width;
    uint16_t height;
    uint8_t format;
    uint8_t reserved[3];
    uint32_t data_size;
    uint32_t crc32;
} ImageHeader;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t max_files;
    uint32_t data_base;
    uint32_t total_size;
    uint32_t sequence;
    uint8_t reserved[44];
} ImageIndexHeader;

typedef struct {
    uint8_t state;
    uint8_t id;
    uint16_t width;
    uint16_t height;
    uint8_t format;
    uint8_t reserved0;
    uint32_t address;
    uint32_t data_size;
    uint32_t crc32;
    uint32_t sequence;
    char name[16];
    uint8_t reserved[20];
} ImageIndexEntry;

typedef struct {
    uint32_t start;
    uint32_t end;
} UsedRange;

static bool gReceiving;
static uint8_t gFileId;
static uint16_t gWidth;
static uint16_t gHeight;
static uint32_t gExpectedSize;
static uint32_t gExpectedCrc;
static uint32_t gReceivedSize;
static uint32_t gRunningCrc;
static uint32_t gWriteAddress;
static uint32_t gDataAddress;
static char gFileName[IMAGE_MAX_NAME_LENGTH + 1U];
static uint8_t gRxBuffer[IMAGE_RX_BUFFER_SIZE];
static size_t gRxLength;
static uint8_t gDisplayBuffer[IMAGE_DISPLAY_BUFFER_SIZE];
static ImageIndexEntry gIndexCompactEntries[IMAGE_MAX_FILES];

static bool start_image_write(uint8_t requestedId, const char *name,
    uint16_t width, uint16_t height, uint32_t size, uint32_t crc32);
static bool erase_range(uint32_t startAddress, uint32_t length);
static void send_page_ack(void);
static void finish_image_write(void);
static uint32_t crc32_update(uint32_t crc, uint8_t byte);
static void send_flash_id(void);
static bool check_flash_ready(void);
static void show_image_id(uint8_t id);
static void show_legacy_slot(uint8_t slot);
static void list_images(void);
static void delete_image_id(uint8_t id);
static void defrag_images(void);
static void send_storage_info(void);
static bool read_index_header(ImageIndexHeader *header);
static bool write_index_header(uint32_t sequence);
static bool ensure_index(void);
static bool read_index_entry(uint8_t index, ImageIndexEntry *entry);
static bool write_index_entry(uint8_t index, const ImageIndexEntry *entry);
static bool find_entry_by_id(uint8_t id, ImageIndexEntry *entry,
    uint8_t *entryIndex);
static bool find_free_entry(uint8_t *entryIndex);
static bool compact_index(void);
static bool rewrite_index_entries(const ImageIndexEntry *entries, uint8_t count);
static bool copy_image_record(uint32_t source, uint32_t destination,
    uint32_t length);
static bool find_free_file_id(uint8_t *id);
static uint32_t next_sequence(void);
static uint32_t align_up_sector(uint32_t value);
static bool find_free_data_range(uint32_t length, uint32_t *address);
static bool ranges_overlap(uint32_t startA, uint32_t endA, uint32_t startB,
    uint32_t endB);
static bool is_valid_entry(const ImageIndexEntry *entry);
static void sanitize_name(const char *source, char *dest, size_t destSize);
static uint32_t legacy_slot_base_address(uint8_t slot);
static bool read_image_header(uint32_t base, ImageHeader *header);
static bool display_image(uint32_t base);

void app_image_store_init(void)
{
    w25q64_init();
}

bool app_image_store_process_command(const char *command)
{
    unsigned int id;
    unsigned int width;
    unsigned int height;
    unsigned long size;
    unsigned long crc;
    char name[IMAGE_MAX_NAME_LENGTH + 1U];

    if (strcmp(command, "FLASHID") == 0) {
        send_flash_id();
        return true;
    }

    if (strcmp(command, "IMG_LIST") == 0) {
        list_images();
        return true;
    }

    if (strcmp(command, "IMG_INFO") == 0) {
        send_storage_info();
        return true;
    }

    if (strcmp(command, "IMG_DEFRAG") == 0) {
        defrag_images();
        return true;
    }

    if (strncmp(command, "IMG_DELETE ", 11U) == 0) {
        if ((sscanf(command + 11, "%u", &id) == 1) && (id < 255U)) {
            delete_image_id((uint8_t) id);
        } else {
            uart_debug_write_string("ERR IMG_DELETE\r\n");
        }
        return true;
    }

    if (strncmp(command, "IMG_SHOW ", 9U) == 0) {
        if ((sscanf(command + 9, "%u", &id) == 1) && (id < 255U)) {
            show_image_id((uint8_t) id);
        } else {
            uart_debug_write_string("ERR IMG_SHOW\r\n");
        }
        return true;
    }

    if (strncmp(command, "IMG_SLOT_SHOW ", 14U) == 0) {
        if (sscanf(command + 14, "%u", &id) == 1) {
            show_legacy_slot((uint8_t) id);
        } else {
            uart_debug_write_string("ERR IMG_SHOW\r\n");
        }
        return true;
    }

    if (strncmp(command, "IMG_SAVE ", 9U) == 0) {
        if (sscanf(command + 9, "%15s %u %u %lu %lx", name, &width, &height,
                &size, &crc) != 5) {
            uart_debug_write_string("ERR IMG_ARGS\r\n");
            return true;
        }

        (void) start_image_write(0xFFU, name, (uint16_t) width,
            (uint16_t) height, (uint32_t) size, (uint32_t) crc);
        return true;
    }

    if (strncmp(command, "IMG_WRITE ", 10U) != 0) {
        return false;
    }

    if ((sscanf(command + 10, "%u %u %u %lu %lx", &id, &width, &height,
            &size, &crc) != 5) || (id >= 255U)) {
        uart_debug_write_string("ERR IMG_ARGS\r\n");
        return true;
    }

    snprintf(name, sizeof(name), "slot%u", id);
    (void) start_image_write((uint8_t) id, name, (uint16_t) width,
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

uint8_t app_image_store_get_file_count(void)
{
    ImageIndexEntry entry;
    uint8_t count = 0U;

    if (!check_flash_ready() || !ensure_index()) {
        return 0U;
    }

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            return count;
        }
        if (is_valid_entry(&entry)) {
            count++;
        }
    }

    return count;
}

bool app_image_store_get_file_by_index(uint8_t listIndex,
    AppImageFileInfo *info)
{
    ImageIndexEntry entry;
    uint8_t count = 0U;

    if ((info == NULL) || !check_flash_ready() || !ensure_index()) {
        return false;
    }

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            return false;
        }
        if (!is_valid_entry(&entry)) {
            continue;
        }
        if (count == listIndex) {
            info->id = entry.id;
            info->width = entry.width;
            info->height = entry.height;
            info->size = entry.data_size;
            memcpy(info->name, entry.name, sizeof(info->name));
            info->name[APP_IMAGE_STORE_NAME_LENGTH] = '\0';
            return true;
        }
        count++;
    }

    return false;
}

bool app_image_store_show_image(uint8_t id)
{
    ImageIndexEntry entry;

    if (!find_entry_by_id(id, &entry, NULL)) {
        return false;
    }

    return display_image(entry.address);
}

static bool start_image_write(uint8_t requestedId, const char *name,
    uint16_t width, uint16_t height, uint32_t size, uint32_t crc32)
{
    uint32_t totalLength = (uint32_t) sizeof(ImageHeader) + size;
    uint32_t address;
    uint8_t id = requestedId;
    char cleanName[IMAGE_MAX_NAME_LENGTH + 1U];

    if (gReceiving || (width == 0U) || (height == 0U) || (size == 0U) ||
        (size != ((uint32_t) width * (uint32_t) height * 2UL)) ||
        ((IMAGE_DATA_BASE + totalLength) > IMAGE_INDEX_BASE)) {
        uart_debug_write_string("ERR IMG_CONFIG\r\n");
        return false;
    }

    if (!check_flash_ready() || !ensure_index()) {
        return false;
    }

    if ((id != 0xFFU) && find_entry_by_id(id, NULL, NULL)) {
        delete_image_id(id);
    }

    if ((id == 0xFFU) && !find_free_file_id(&id)) {
        uart_debug_write_string("ERR IMG_NO_ID\r\n");
        return false;
    }

    if (!find_free_data_range(totalLength, &address)) {
        uart_debug_write_string("ERR IMG_NO_SPACE\r\n");
        return false;
    }

    uart_debug_write_string("OK IMG_ERASE\r\n");
    if (!erase_range(address, totalLength)) {
        uart_debug_write_string("ERR IMG_ERASE\r\n");
        return false;
    }

    sanitize_name(name, cleanName, sizeof(cleanName));
    gReceiving = true;
    gFileId = id;
    gWidth = width;
    gHeight = height;
    gExpectedSize = size;
    gExpectedCrc = crc32;
    gReceivedSize = 0U;
    gRunningCrc = 0xFFFFFFFFUL;
    gDataAddress = address;
    gWriteAddress = address + (uint32_t) sizeof(ImageHeader);
    gRxLength = 0U;
    strncpy(gFileName, cleanName, sizeof(gFileName));
    gFileName[sizeof(gFileName) - 1U] = '\0';
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
    uint8_t entryIndex;
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
    ImageIndexEntry entry;

    gReceiving = false;
    if (finalCrc != gExpectedCrc) {
        uart_debug_write_string("ERR IMG_CRC\r\n");
        return;
    }

    if (!w25q64_write(gDataAddress, (const uint8_t *) &header, sizeof(header))) {
        uart_debug_write_string("ERR IMG_HEADER\r\n");
        return;
    }

    if (!find_free_entry(&entryIndex)) {
        if (!compact_index() || !find_free_entry(&entryIndex)) {
            uart_debug_write_string("ERR IMG_INDEX_FULL\r\n");
            return;
        }
    }

    memset(&entry, 0xFF, sizeof(entry));
    entry.state = IMAGE_STATE_ACTIVE;
    entry.id = gFileId;
    entry.width = gWidth;
    entry.height = gHeight;
    entry.format = IMAGE_FORMAT_RGB565;
    entry.address = gDataAddress;
    entry.data_size = gExpectedSize;
    entry.crc32 = finalCrc;
    entry.sequence = next_sequence();
    strncpy(entry.name, gFileName, sizeof(entry.name));
    entry.name[sizeof(entry.name) - 1U] = '\0';

    if (!write_index_entry(entryIndex, &entry)) {
        uart_debug_write_string("ERR IMG_INDEX\r\n");
        return;
    }

    snprintf(message, sizeof(message), "OK IMG_DONE ID=%u SIZE=%lu CRC=%08lX\r\n",
        gFileId, (unsigned long) gExpectedSize, (unsigned long) finalCrc);
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

static void show_image_id(uint8_t id)
{
    ImageIndexEntry entry;

    if (find_entry_by_id(id, &entry, NULL)) {
        if (display_image(entry.address)) {
            uart_debug_write_string("OK IMG_SHOW\r\n");
        }
        return;
    }

    if (id < IMAGE_LEGACY_MAX_SLOTS) {
        show_legacy_slot(id);
        return;
    }

    uart_debug_write_string("ERR IMG_NOT_FOUND\r\n");
}

static void show_legacy_slot(uint8_t slot)
{
    uint32_t base;

    if (slot >= IMAGE_LEGACY_MAX_SLOTS) {
        uart_debug_write_string("ERR IMG_SLOT\r\n");
        return;
    }

    base = legacy_slot_base_address(slot);
    if (display_image(base)) {
        uart_debug_write_string("OK IMG_SHOW\r\n");
    }
}

static void list_images(void)
{
    char message[128];
    ImageIndexEntry entry;
    uint8_t count = 0U;

    if (!check_flash_ready() || !ensure_index()) {
        return;
    }

    uart_debug_write_string("IMG_LIST_BEGIN\r\n");
    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            uart_debug_write_string("ERR IMG_INDEX_READ\r\n");
            return;
        }
        if (!is_valid_entry(&entry)) {
            continue;
        }
        snprintf(message, sizeof(message),
            "IMG_FILE ID=%u NAME=%s W=%u H=%u SIZE=%lu CRC=%08lX ADDR=%06lX\r\n",
            entry.id, entry.name, entry.width, entry.height,
            (unsigned long) entry.data_size, (unsigned long) entry.crc32,
            (unsigned long) entry.address);
        uart_debug_write_string(message);
        count++;
    }
    snprintf(message, sizeof(message), "IMG_LIST_END COUNT=%u\r\n", count);
    uart_debug_write_string(message);
}

static void delete_image_id(uint8_t id)
{
    char message[64];
    ImageIndexEntry entry;
    uint8_t entryIndex;
    uint32_t totalLength;

    if (!check_flash_ready() || !ensure_index()) {
        return;
    }

    if (!find_entry_by_id(id, &entry, &entryIndex)) {
        uart_debug_write_string("ERR IMG_NOT_FOUND\r\n");
        return;
    }

    totalLength = (uint32_t) sizeof(ImageHeader) + entry.data_size;
    if (!erase_range(entry.address, totalLength)) {
        uart_debug_write_string("ERR IMG_ERASE\r\n");
        return;
    }

    entry.state = IMAGE_STATE_DELETED;
    if (!write_index_entry(entryIndex, &entry)) {
        uart_debug_write_string("ERR IMG_INDEX\r\n");
        return;
    }

    snprintf(message, sizeof(message), "OK IMG_DELETE ID=%u\r\n", id);
    uart_debug_write_string(message);
}

static void defrag_images(void)
{
    char message[96];
    ImageIndexEntry entry;
    uint8_t count = 0U;
    uint8_t moved = 0U;
    uint32_t cursor = IMAGE_DATA_BASE;

    if (gReceiving) {
        uart_debug_write_string("ERR IMG_BUSY\r\n");
        return;
    }

    if (!check_flash_ready() || !ensure_index()) {
        return;
    }

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            uart_debug_write_string("ERR IMG_INDEX_READ\r\n");
            return;
        }
        if (is_valid_entry(&entry)) {
            gIndexCompactEntries[count++] = entry;
        }
    }

    for (uint8_t i = 0U; i < count; i++) {
        for (uint8_t j = (uint8_t) (i + 1U); j < count; j++) {
            if (gIndexCompactEntries[j].address <
                gIndexCompactEntries[i].address) {
                ImageIndexEntry temp = gIndexCompactEntries[i];
                gIndexCompactEntries[i] = gIndexCompactEntries[j];
                gIndexCompactEntries[j] = temp;
            }
        }
    }

    uart_debug_write_string("OK IMG_DEFRAG_BEGIN\r\n");
    for (uint8_t i = 0U; i < count; i++) {
        uint32_t length = (uint32_t) sizeof(ImageHeader) +
            gIndexCompactEntries[i].data_size;
        uint32_t alignedLength = align_up_sector(length);

        if (cursor != gIndexCompactEntries[i].address) {
            if (!copy_image_record(gIndexCompactEntries[i].address, cursor,
                    length)) {
                uart_debug_write_string("ERR IMG_DEFRAG_COPY\r\n");
                return;
            }
            gIndexCompactEntries[i].address = cursor;
            moved++;
        }

        cursor += alignedLength;
        if (cursor > IMAGE_INDEX_BASE) {
            uart_debug_write_string("ERR IMG_DEFRAG_SPACE\r\n");
            return;
        }
    }

    if (!rewrite_index_entries(gIndexCompactEntries, count)) {
        uart_debug_write_string("ERR IMG_INDEX\r\n");
        return;
    }

    snprintf(message, sizeof(message),
        "OK IMG_DEFRAG MOVED=%u COUNT=%u END=%06lX\r\n", moved, count,
        (unsigned long) cursor);
    uart_debug_write_string(message);
}

static void send_storage_info(void)
{
    char message[96];
    ImageIndexEntry entry;
    uint32_t used = 0UL;
    uint8_t count = 0U;

    if (!check_flash_ready() || !ensure_index()) {
        return;
    }

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            uart_debug_write_string("ERR IMG_INDEX_READ\r\n");
            return;
        }
        if (!is_valid_entry(&entry)) {
            continue;
        }
        used += align_up_sector((uint32_t) sizeof(ImageHeader) +
            entry.data_size);
        count++;
    }

    snprintf(message, sizeof(message),
        "IMG_INFO TOTAL=%lu USED=%lu FREE=%lu COUNT=%u\r\n",
        (unsigned long) (IMAGE_INDEX_BASE - IMAGE_DATA_BASE),
        (unsigned long) used,
        (unsigned long) ((IMAGE_INDEX_BASE - IMAGE_DATA_BASE) - used),
        count);
    uart_debug_write_string(message);
}

static bool read_index_header(ImageIndexHeader *header)
{
    return w25q64_read(IMAGE_INDEX_BASE, (uint8_t *) header, sizeof(*header));
}

static bool write_index_header(uint32_t sequence)
{
    ImageIndexHeader header;

    memset(&header, 0xFF, sizeof(header));
    header.magic = IMAGE_INDEX_MAGIC;
    header.version = IMAGE_INDEX_VERSION;
    header.max_files = IMAGE_MAX_FILES;
    header.data_base = IMAGE_DATA_BASE;
    header.total_size = IMAGE_INDEX_BASE;
    header.sequence = sequence;

    return w25q64_write(IMAGE_INDEX_BASE, (const uint8_t *) &header,
        sizeof(header));
}

static bool ensure_index(void)
{
    ImageIndexHeader header;

    if (!read_index_header(&header)) {
        uart_debug_write_string("ERR IMG_INDEX_READ\r\n");
        return false;
    }

    if ((header.magic == IMAGE_INDEX_MAGIC) &&
        (header.version == IMAGE_INDEX_VERSION) &&
        (header.max_files == IMAGE_MAX_FILES) &&
        (header.data_base == IMAGE_DATA_BASE)) {
        return true;
    }

    for (uint32_t i = 0U; i < IMAGE_INDEX_SECTOR_COUNT; i++) {
        if (!w25q64_erase_sector(IMAGE_INDEX_BASE +
                (i * W25Q64_SECTOR_SIZE_BYTES))) {
            uart_debug_write_string("ERR IMG_INDEX_ERASE\r\n");
            return false;
        }
    }

    if (!write_index_header(0UL)) {
        uart_debug_write_string("ERR IMG_INDEX\r\n");
        return false;
    }

    uart_debug_write_string("OK IMG_INDEX_INIT\r\n");
    return true;
}

static bool read_index_entry(uint8_t index, ImageIndexEntry *entry)
{
    uint32_t address = IMAGE_INDEX_BASE + sizeof(ImageIndexHeader) +
        ((uint32_t) index * (uint32_t) sizeof(ImageIndexEntry));

    if (index >= IMAGE_MAX_FILES) {
        return false;
    }

    return w25q64_read(address, (uint8_t *) entry, sizeof(*entry));
}

static bool write_index_entry(uint8_t index, const ImageIndexEntry *entry)
{
    uint32_t address = IMAGE_INDEX_BASE + sizeof(ImageIndexHeader) +
        ((uint32_t) index * (uint32_t) sizeof(ImageIndexEntry));

    if (index >= IMAGE_MAX_FILES) {
        return false;
    }

    return w25q64_write(address, (const uint8_t *) entry, sizeof(*entry));
}

static bool find_entry_by_id(uint8_t id, ImageIndexEntry *entry,
    uint8_t *entryIndex)
{
    ImageIndexEntry candidate;

    if (!ensure_index()) {
        return false;
    }

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &candidate)) {
            return false;
        }
        if (is_valid_entry(&candidate) && (candidate.id == id)) {
            if (entry != NULL) {
                *entry = candidate;
            }
            if (entryIndex != NULL) {
                *entryIndex = i;
            }
            return true;
        }
    }

    return false;
}

static bool find_free_entry(uint8_t *entryIndex)
{
    ImageIndexEntry entry;

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            return false;
        }
        if (entry.state == IMAGE_STATE_EMPTY) {
            *entryIndex = i;
            return true;
        }
    }

    return false;
}

static bool compact_index(void)
{
    ImageIndexEntry entry;
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            return false;
        }
        if (is_valid_entry(&entry)) {
            gIndexCompactEntries[count++] = entry;
        }
    }

    if (rewrite_index_entries(gIndexCompactEntries, count)) {
        uart_debug_write_string("OK IMG_INDEX_COMPACT\r\n");
        return true;
    }

    return false;
}

static bool rewrite_index_entries(const ImageIndexEntry *entries, uint8_t count)
{
    if ((entries == NULL) && (count > 0U)) {
        return false;
    }

    for (uint32_t i = 0U; i < IMAGE_INDEX_SECTOR_COUNT; i++) {
        if (!w25q64_erase_sector(IMAGE_INDEX_BASE +
                (i * W25Q64_SECTOR_SIZE_BYTES))) {
            return false;
        }
    }

    if (!write_index_header(0UL)) {
        return false;
    }

    for (uint8_t i = 0U; i < count; i++) {
        if (!write_index_entry(i, &entries[i])) {
            return false;
        }
    }

    return true;
}

static bool copy_image_record(uint32_t source, uint32_t destination,
    uint32_t length)
{
    uint32_t copied = 0UL;

    if ((source == destination) || (length == 0UL)) {
        return true;
    }

    if ((source < IMAGE_DATA_BASE) || (destination < IMAGE_DATA_BASE) ||
        ((source + length) > IMAGE_INDEX_BASE) ||
        ((destination + length) > IMAGE_INDEX_BASE)) {
        return false;
    }

    if (!erase_range(destination, length)) {
        return false;
    }

    while (copied < length) {
        size_t chunk = sizeof(gDisplayBuffer);
        if (chunk > (length - copied)) {
            chunk = (size_t) (length - copied);
        }

        if (!w25q64_read(source + copied, gDisplayBuffer, chunk)) {
            return false;
        }
        if (!w25q64_write(destination + copied, gDisplayBuffer, chunk)) {
            return false;
        }
        copied += (uint32_t) chunk;
    }

    return true;
}

static bool find_free_file_id(uint8_t *id)
{
    for (uint16_t candidate = 0U; candidate < 255U; candidate++) {
        if (!find_entry_by_id((uint8_t) candidate, NULL, NULL)) {
            *id = (uint8_t) candidate;
            return true;
        }
    }

    return false;
}

static uint32_t next_sequence(void)
{
    ImageIndexEntry entry;
    uint32_t maxSequence = 0UL;

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            return 1UL;
        }
        if (is_valid_entry(&entry) && (entry.sequence > maxSequence)) {
            maxSequence = entry.sequence;
        }
    }

    return maxSequence + 1UL;
}

static uint32_t align_up_sector(uint32_t value)
{
    return (value + W25Q64_SECTOR_SIZE_BYTES - 1UL) &
        ~(W25Q64_SECTOR_SIZE_BYTES - 1UL);
}

static bool find_free_data_range(uint32_t length, uint32_t *address)
{
    UsedRange ranges[IMAGE_MAX_FILES];
    ImageIndexEntry entry;
    uint8_t rangeCount = 0U;
    uint32_t alignedLength = align_up_sector(length);
    uint32_t cursor = IMAGE_DATA_BASE;
    bool moved;

    for (uint8_t i = 0U; i < IMAGE_MAX_FILES; i++) {
        if (!read_index_entry(i, &entry)) {
            return false;
        }
        if (!is_valid_entry(&entry)) {
            continue;
        }
        ranges[rangeCount].start = entry.address;
        ranges[rangeCount].end = entry.address + align_up_sector(
            (uint32_t) sizeof(ImageHeader) + entry.data_size);
        rangeCount++;
    }

    do {
        uint32_t candidateEnd = cursor + alignedLength;
        moved = false;

        if (candidateEnd > IMAGE_INDEX_BASE) {
            return false;
        }

        for (uint8_t i = 0U; i < rangeCount; i++) {
            if (ranges_overlap(cursor, candidateEnd, ranges[i].start,
                    ranges[i].end)) {
                cursor = ranges[i].end;
                moved = true;
                break;
            }
        }
    } while (moved);

    *address = cursor;
    return true;
}

static bool ranges_overlap(uint32_t startA, uint32_t endA, uint32_t startB,
    uint32_t endB)
{
    return (startA < endB) && (startB < endA);
}

static bool is_valid_entry(const ImageIndexEntry *entry)
{
    if ((entry->state != IMAGE_STATE_ACTIVE) ||
        (entry->format != IMAGE_FORMAT_RGB565) ||
        (entry->width == 0U) || (entry->height == 0U) ||
        (entry->address < IMAGE_DATA_BASE) ||
        (entry->address >= IMAGE_INDEX_BASE) ||
        (entry->data_size !=
            ((uint32_t) entry->width * (uint32_t) entry->height * 2UL)) ||
        ((entry->address + sizeof(ImageHeader) + entry->data_size) >
            IMAGE_INDEX_BASE)) {
        return false;
    }

    return true;
}

static void sanitize_name(const char *source, char *dest, size_t destSize)
{
    size_t out = 0U;

    if (destSize == 0U) {
        return;
    }

    while ((source != NULL) && (*source != '\0') && (out < (destSize - 1U))) {
        char ch = *source++;
        if (((ch >= 'A') && (ch <= 'Z')) || ((ch >= 'a') && (ch <= 'z')) ||
            ((ch >= '0') && (ch <= '9')) || (ch == '_') || (ch == '-') ||
            (ch == '.')) {
            dest[out++] = ch;
        }
    }

    if (out == 0U) {
        strncpy(dest, "image", destSize);
        dest[destSize - 1U] = '\0';
        return;
    }

    dest[out] = '\0';
}

static uint32_t legacy_slot_base_address(uint8_t slot)
{
    return IMAGE_LEGACY_SLOT_BASE + ((uint32_t) slot * IMAGE_LEGACY_SLOT_SIZE);
}

static bool read_image_header(uint32_t base, ImageHeader *header)
{
    if (!w25q64_read(base, (uint8_t *) header, sizeof(*header))) {
        uart_debug_write_string("ERR IMG_READ_HEADER\r\n");
        return false;
    }

    if ((header->magic != IMAGE_MAGIC) ||
        (header->format != IMAGE_FORMAT_RGB565) ||
        (header->width == 0U) ||
        (header->height == 0U) ||
        (header->data_size !=
            ((uint32_t) header->width * (uint32_t) header->height * 2UL)) ||
        ((base + sizeof(ImageHeader) + header->data_size) >
            IMAGE_INDEX_BASE)) {
        uart_debug_write_string("ERR IMG_HEADER_INVALID\r\n");
        return false;
    }

    return true;
}

static bool display_image(uint32_t base)
{
    ImageHeader header;
    uint32_t dataAddress;
    uint32_t remaining;

    if (!read_image_header(base, &header)) {
        return false;
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
            return false;
        }
        for (size_t i = 0U; i < chunk; i++) {
            LCD_WR_DATA8(gDisplayBuffer[i]);
        }
        dataAddress += (uint32_t) chunk;
        remaining -= (uint32_t) chunk;
    }

    return true;
}
