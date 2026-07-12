#ifndef APP_MOTION_CONTROL_H
#define APP_MOTION_CONTROL_H

#include "hw/hw_encoder.h"
#include <stdbool.h>
#include <stdint.h>

void motion_control_init(void);
void motion_control_set_velocity(float linearMmS, float angularDegS);
void motion_control_set_wheel_targets(float leftCountsPerPeriod,
    float rightCountsPerPeriod);
bool motion_control_parse_command(const char *command, EncoderCounts counts);
void motion_control_update(EncoderCounts counts, float *leftTargetCounts,
    float *rightTargetCounts);
bool motion_control_is_active(void);
bool motion_control_is_busy(void);
int32_t motion_control_get_mode(void);
int32_t motion_control_get_target_mm_s(void);
int32_t motion_control_get_target_deg_s(void);
float motion_control_get_left_target_counts(void);
float motion_control_get_right_target_counts(void);

#endif
