#ifndef HW_OPENMV_UART_H
#define HW_OPENMV_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UART_OPENMV_LINE_BUFFER_SIZE (32U)

void uart_openmv_init(void);
void uart_openmv_write_string(const char *text);
bool uart_openmv_read_line(char *line, size_t lineSize);
void uart_openmv_handle_irq(void);

#endif
