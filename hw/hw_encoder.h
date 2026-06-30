#ifndef HW_ENCODER_H
#define HW_ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

typedef struct {
    int32_t left_count;
    int32_t right_count;
} EncoderCounts;

void encoder_init(void);
EncoderCounts encoder_get_counts(void);
void encoder_reset(void);
void encoder_handle_gpio_irq(void);

#endif
