#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdint.h>
#include "defines.h"

#ifdef WTM_PROG_SPI
	#include "driver/spi_master.h"
    void spiCopyHandleForExp(spi_device_handle_t handle);
    void spiSetBusAvailable(bool available);
#endif

void mcp23017_init(int fd);

void bus_release(int iic);

int flash_chip_erase(int iic);

uint16_t flash_read_product_id(int iic);

uint8_t bus_read_cycle(int iic, uint16_t addr);

int flash_program_byte(int iic, uint16_t addr, uint8_t data);

// Want to get rid of this
int goProgrammer();

#endif /* __FLASH_H__ */