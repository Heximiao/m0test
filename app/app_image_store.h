#ifndef APP_IMAGE_STORE_H
#define APP_IMAGE_STORE_H

#include <stdbool.h>

void app_image_store_init(void);
bool app_image_store_process_command(const char *command);
bool app_image_store_is_receiving(void);
void app_image_store_service(void);

#endif
