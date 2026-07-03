#ifndef APP_MPU6050_ATTITUDE_H
#define APP_MPU6050_ATTITUDE_H

#include <stdbool.h>

void app_mpu6050_attitude_init(void);
bool app_mpu6050_attitude_process_command(const char *command);
void app_mpu6050_attitude_send(void);

#endif
