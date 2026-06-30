#ifndef APP_CAR_CONTROL_H
#define APP_CAR_CONTROL_H

#include <stdint.h>

void app_car_control_init(void);
void app_car_control_update(uint32_t nowMs);
void app_car_control_process_command(char *command);
void app_car_control_send_telemetry(void);
void app_car_control_send_heartbeat(uint32_t nowMs);
void app_car_control_toggle_status_led(void);

#endif
