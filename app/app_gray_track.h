#ifndef APP_GRAY_TRACK_H
#define APP_GRAY_TRACK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    int32_t error;
    int16_t turn_angle_deg;
    uint8_t sensor_bits;
} GrayTrackResult;

void gray_track_init(void);
void gray_track_set_enabled(bool enabled);
bool gray_track_is_enabled(void);
GrayTrackResult gray_track_update(uint32_t nowMs);
uint8_t gray_track_get_level_bits(void);

#endif
