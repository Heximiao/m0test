#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

void line_follow_init(void);
bool line_follow_parse_command(const char *command, uint32_t nowMs);
float line_follow_get_turn_adjust(uint32_t nowMs);
bool line_follow_is_active(uint32_t nowMs);
bool line_follow_is_valid(uint32_t nowMs);
int32_t line_follow_get_error(void);

#endif
