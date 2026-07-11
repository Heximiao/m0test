#ifndef HW_W25Q64_H
#define HW_W25Q64_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define W25Q64_TOTAL_SIZE_BYTES (8UL * 1024UL * 1024UL)
#define W25Q64_SECTOR_SIZE_BYTES (4096UL)
#define W25Q64_PAGE_SIZE_BYTES (256UL)

void w25q64_init(void);
uint32_t w25q64_read_jedec_id(void);
uint32_t w25q64_read_jedec_id_falling_sample(void);
uint32_t w25q64_read_jedec_id_atomic(void);
uint32_t w25q64_read_gpio_levels(void);
void w25q64_force_cs_level(bool high);
void w25q64_force_clk_level(bool high);
void w25q64_force_mosi_level(bool high);
bool w25q64_read_miso_level(void);
void w25q64_probe_jedec_ids(uint32_t *normalId, uint32_t *swappedId);
bool w25q64_is_valid_jedec_id(uint32_t id);
bool w25q64_erase_sector(uint32_t address);
bool w25q64_read(uint32_t address, uint8_t *buffer, size_t length);
bool w25q64_write(uint32_t address, const uint8_t *buffer, size_t length);

#endif
