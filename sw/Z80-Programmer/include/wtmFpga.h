/*************************************************************************
 * 
 *    Code & ESP Programmer Board by Denno Wiggle aka WTM
 * 
 *    This is code to communicate with an FPGA and it's flash memory
 *    
 *************************************************************************
 *
 *    This library is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 2.1 of the License, or (at your option) any later version.
 *
 *    This library is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with this library; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *    USA
 *
 *****************************************************************************/
#ifndef __WTM_FPGA_H__
#define __WTM_FPGA_H__

#define PAGE_BLOCK_SIZE (0x100)        // block size for accessing the flash = 256 bytes

#include "driver/spi_master.h"

void initFpgaFlash();
void deinitFpgaFlash();
void spiCopyHandleForFpgaFlash(spi_device_handle_t handle);
int eraseFpgaFlash(uint64_t flashAddr, uint64_t flashSize, uint64_t size);
esp_err_t readFpgaFlash(uint32_t flashAddr, uint8_t* data, const uint16_t dataLen);
void writeFpgaFlash(uint64_t flashAddr, uint8_t* data, uint16_t dataLen);
bool readFpgaFlashId();
void testFpgaFlash();

#endif /* __WTM_FPGA_H__ */