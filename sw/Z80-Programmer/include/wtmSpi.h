/*************************************************************************
 * 
 *    Code & ESP Programmer Board by Denno Wiggle aka WTM
 * 
 *    This is code to control the SPI bus.
 * 
 *    The ESP32 has two SPI busses available to the user.
 *    Bus 2 -  is used to communicate with MPC23S17
 *             and the SD card in SPI mode
 *    Bus 3 - is used for auxillary (AUX) use such as for an FPGA.
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
 *****************************************************************************
 *
 *    Espressif Documentation:
 *    Ref "SPI Master Driver"
 *    https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_master.html
 *    Ref "SD SPI Host Driver"
 *    https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/sdspi_host.html
 * 
 *****************************************************************************/

#ifndef __WTM_SPI_H__
#define __WTM_SPI_H__

#include "esp_err.h"

esp_err_t initSpi();
esp_err_t initSpiMain();
esp_err_t initSpiAux();

esp_err_t deinitSpiMain();
esp_err_t deinitSpiAux();
void deinitSpi();


bool spiSdCardStatus();
void handleSdCardEvents();

// For test purposes
// Todo : delete
esp_err_t lowLevelSpiTest(bool incSdCardTest);

#endif /* __WTM_SPI_H__ */