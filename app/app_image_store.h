#ifndef APP_IMAGE_STORE_H
#define APP_IMAGE_STORE_H

#include <stdbool.h>
#include <stdint.h>

#define APP_IMAGE_STORE_NAME_LENGTH (15U)

typedef struct {
    uint8_t id;
    uint16_t width;
    uint16_t height;
    uint32_t size;
    char name[APP_IMAGE_STORE_NAME_LENGTH + 1U];
} AppImageFileInfo;

void app_image_store_init(void);
bool app_image_store_process_command(const char *command);
bool app_image_store_is_receiving(void);
void app_image_store_service(void);
uint8_t app_image_store_get_file_count(void);
bool app_image_store_get_file_by_index(uint8_t listIndex,
    AppImageFileInfo *info);
bool app_image_store_show_image(uint8_t id);

#endif
