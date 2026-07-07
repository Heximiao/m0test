#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

void app_attitude_init(void);
bool app_attitude_process_command(const char *command);
void app_attitude_send(uint32_t nowMs);
void app_attitude_poll(void);

#endif
