#ifndef HW_UART_H
#define HW_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UART_DEBUG_LINE_BUFFER_SIZE (64U)

void uart_debug_init(void);
void uart_debug_service_tx(void);
void uart_debug_write_string(const char *text);
bool uart_debug_read_line(char *line, size_t lineSize);
bool uart_debug_read_byte(uint8_t *byte);
void uart_debug_handle_irq(void);
uint32_t uart_debug_get_rx_dropped_bytes(void);
uint32_t uart_debug_get_tx_dropped_bytes(void);

#endif
