#include "hw_spi.h"

void spi_write_bus(unsigned char dat)
{
    DL_SPI_transmitData8(SPI_LCD_INST, dat);
    while (DL_SPI_isBusy(SPI_LCD_INST)) {
    }
}
