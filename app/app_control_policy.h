#ifndef APP_CONTROL_POLICY_H
#define APP_CONTROL_POLICY_H

#include <stdbool.h>

void control_policy_init(void);
bool control_policy_start_line_follow(void);
void control_policy_stop_line_follow(void);
bool control_policy_is_line_follow_requested(void);
bool control_policy_can_run_line_follow(void);
void control_policy_set_gray_enabled(bool enabled);
bool control_policy_is_gray_enabled(void);
void control_policy_set_gyro_enabled(bool enabled);
bool control_policy_is_gyro_enabled(void);
void control_policy_set_navigation_active(bool active);
bool control_policy_is_navigation_active(void);

#endif
