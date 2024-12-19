/*************************************************************************
 * 
 *    Code & ESP Programmer Board by Denno Wiggle aka WTM
 * 
 *    This is code to control the SPI bus.
 * 
 *    The ESP32 has two SPI busses available to the user.
 *    Bus 3 - is used to communicate with MPC23S17
 *            and the SD card in SPI mode
 *    Bus 4 - is used for auxillary (AUX) use such as for an FPGA.
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


#include <stdio.h>
// Can we remove these three.
// #include <stdlib.h>
// #include "FS.h"
// #include "SD.h"

#include "SPI.h"
#include "driver/spi_master.h"

#include "defines.h"
// Todo : Can we delete this
// #include "sd_diskio.h"

#include "driver/sdspi_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_common.h"
// #include "sdmmc_cmd.h"
#include "wtmSerialx.h"
#include "dbg.h"
// Todo : Must be a better way than needing spiCopyHandleForExp() in flash.h
#include "flash.h"
#include "wtmFpga.h"

extern wtmSerialX* console;
extern wtmSerialX* logger;

// Custom structure to hold the configuration of a single bus. Implementing
// it this way allows us to pass the parameters easily between functions.
struct spiBusCfg {
    spi_host_device_t host;
    int8_t clk;
    int8_t mosi;
    int8_t miso;
    int8_t cs0;
    int8_t cs1;
    int8_t sdDet;
    uint16_t freqkhz;
    sdmmc_host_t sdHost;
} typedef wtmSpiBusPinCfg;

// Two global structure variables to hold the configuration of the two 
// user SPI buses. 
wtmSpiBusPinCfg spiPinCfgSdExp;
wtmSpiBusPinCfg spiPinCfgAux;

// Declare the handles used for accessing the three devices on the bus
// 1. MCP23S17 x 2 Input/Output Expander IC's.
// 2. SD card (low level access)
// 3. Aux port for off-board SPI (e.g. FPGA board)
spi_device_handle_t ioExpanderHandle;
sdspi_dev_handle_t sdHandle;
spi_device_handle_t auxHandle;

// Structure to hold the parametrs of an SD card whhich are filled during
// initialisation by espressif ESP32 SW. Higher level SW uses this 
// rather than the handle.
sdmmc_card_t* sdCard;

// Declare the two SPI busses and SPI host device 
// 1. Bus for onboard devices, the SD card and IO Expander IC's
// 2. Bus for off-board use such as to an FPGA.
SPIClass spiBusSd = SPIClass(WTM_SPI_HOST_SD);
SPIClass spiBusAux = SPIClass(WTM_SPI_HOST_AUX);
spi_host_device_t hostSdAndIoExp = WTM_SPI_HOST_SD;
spi_host_device_t hostAux = WTM_SPI_HOST_AUX;

/**
 * @brief SD card information structure
 * The SD card information structure variable is used to hold the card 
 * detected status of the SD card for handling plug and unplug events and 
 * whether the SD card needs to be initialised again.
 *****************************************************************************/
struct wtmSdCardInfo {
	const uint8_t detectPin;
	bool cardDetected;
    bool cardDetectPrev;
    bool initDone;
};

/**
 * @brief SD card information structure variable.
 * The SD card information structure variable is used to hold the card 
 * detected status of the SD card for handling plug and unplug events and 
 * whether the SD card needs to be initialised again.
 * The initial values are set here.
 *****************************************************************************/
wtmSdCardInfo sdCardInfo = {
    .detectPin = WTM_SPI_SD_CD,
    .cardDetected = false,
    .cardDetectPrev = false,
    .initDone = false,
};

/**
 * @brief Initialisation status of the SPI bbus.
 * An 8-bit status value to log which parts of the two SPI busses have
 * been initialised.
 *****************************************************************************/
uint8_t spiStatus;

// Bit values for use with the 8-but status value for initialisation.
#define STATUS_INIT_BUS_SD   (1<<0)
#define STATUS_INIT_SD       (1<<1)
#define STATUS_INIT_IO_EXP   (1<<2)
#define STATUS_INIT_SD_CARD  (1<<3)
#define STATUS_INIT_BUS_AUX  (1<<4)
#define STATUS_INIT_AUX_CS   (1<<5)


/**
 * A helper function to log print the esp_err_t type SPI error codes that 
 * occur during initialisation.
 * @param errCode holds the error code
 *****************************************************************************/
void spiDecodeError(esp_err_t errCode)
{

    const size_t buffSize = (size_t)100;
    char strBuffer[buffSize];
    snprintf(strBuffer, buffSize, "WIGGLEIA_HOME");

    switch(errCode)
    {
        case ESP_OK:
            snprintf(strBuffer, buffSize, "...ESP_OK = Success\r\n");
            break;
        case ESP_FAIL:
            snprintf(strBuffer, buffSize, "...ESP_FAIL = Failure\r\n");
            break;
        case ESP_ERR_NO_MEM:
            snprintf(strBuffer, buffSize, "...ESP_ERR_NO_MEM = Out of memory\r\n");
            break;
        case ESP_ERR_INVALID_ARG:
            snprintf(strBuffer, buffSize, "...ESP_ERR_INVALID_ARG = Invalid argument\r\n");
            break;
        case ESP_ERR_INVALID_STATE:
            snprintf(strBuffer, buffSize, "...ESP_ERR_INVALID_STATE = Invalid state\r\n");
            break;
        case ESP_ERR_INVALID_SIZE:
            snprintf(strBuffer, buffSize, "...ESP_ERR_INVALID_SIZE = Invalid size\r\n");
            break;
        case ESP_ERR_NOT_FOUND:
            snprintf(strBuffer, buffSize, "...ESP_ERR_NOT_FOUND = Requested resource not found\r\n");
            break;
        case ESP_ERR_NOT_SUPPORTED:
            snprintf(strBuffer, buffSize, "...ESP_ERR_NOT_SUPPORTED = peration or feature not supported\r\n");
            break;
        case ESP_ERR_TIMEOUT:
            snprintf(strBuffer, buffSize, "...ESP_ERR_TIMEOUT = Operation timed out\r\n");
            break;
        case ESP_ERR_INVALID_RESPONSE:
            snprintf(strBuffer, buffSize, "...ESP_ERR_INVALID_RESPONSE = Received response was invalid\r\n");
            break;
        case ESP_ERR_INVALID_CRC:
            snprintf(strBuffer, buffSize, "...ESP_ERR_INVALID_CRC = CRC or checksum was invalid\r\n");
            break;
        case ESP_ERR_INVALID_VERSION:
            snprintf(strBuffer, buffSize, "...ESP_ERR_INVALID_VERSION = Version was invalid\r\n");
            break;
        case ESP_ERR_INVALID_MAC:
            snprintf(strBuffer, buffSize, "...ESP_ERR_INVALID_MAC = MAC address was invalid\r\n");
            break;
        case ESP_ERR_NOT_FINISHED:
            snprintf(strBuffer, buffSize, "...ESP_ERR_NOT_FINISHED = There are items remained to retrieve\r\n");
            break;
        default:
            snprintf(strBuffer, buffSize, "...Detail not available for error code = 0x%04x\r\n", errCode);
            break;
    }
    logger->write(strBuffer, buffSize);
#ifdef WTM_DBG_SPI_INIT
    console->write(strBuffer, buffSize);
#endif
}

/**
 * @brief Interrupt service routine for the SD card detect pin.
 * When a change occurs in the level of the card detect pin we need to set a flag
 * for later action to initialise the card.
 *****************************************************************************/
void IRAM_ATTR sdCardDetectIsr()
{
    if (digitalRead(sdCardInfo.detectPin) == 1)
    {
        sdCardInfo.cardDetected = false;
        sdCardInfo.initDone = false;
    } else {
        sdCardInfo.cardDetected = true;
        // set the SPI bus availability for MCP23S17 devices on the bus to 
        // false until SD card init is done, otherwise the SD card might not
        // go into SPI mode according to the espressif docs.
        spiSetBusAvailable(false);
    }
}

/**
 * @brief SD card detect pin initialisation.
 * Sets the state of program flags relating to the SD card detect pin.
 * The sdCardInfo global variable hold flags for the plugged in state 
 * of the SD card and ultimately facilitates plug and unplug code.
 *****************************************************************************/
void sdCardDetectPinSetup()
{
    pinMode(sdCardInfo.detectPin, INPUT_PULLUP);
    uint8_t readPin = digitalRead(sdCardInfo.detectPin);
    if (readPin == 0x01)
    {
        sdCardInfo.cardDetected = false;
        sdCardInfo.cardDetectPrev = false;
    } else {
        sdCardInfo.cardDetected = true;
        sdCardInfo.cardDetectPrev = true;
    }
    logger->printf("...SD card is %s\r\n", sdCardInfo.cardDetected? "detected" : "not detected");
#ifdef WTM_DBG_SPI_INIT
    console->printf("...SD card is %s\r\n", sdCardInfo.cardDetected? "detected" : "not detected");
#endif
    // Attach the interrupt service routine to the SD card detect pin
    attachInterrupt(sdCardInfo.detectPin, sdCardDetectIsr, CHANGE);
}


/**
 * @brief Print out information about an SD card. 
 * Overrides sdmmc_card_print_info() function so we can redirect the output.
 * @param port defines where to send the output.
 * @param card is the structure that has information about the card. The info
 * in the structure was found during card initialisation.
 *****************************************************************************/
void spiSdCardPrintInfo(wtmSerialX* port, const sdmmc_card_t* card)
{
    bool print_scr = false;
    bool print_csd = false;
    const char* type;
    port->printf("Name: %s\r\n", card->cid.name);
    if (card->is_sdio) {
        type = "SDIO";
        print_scr = true;
        print_csd = true;
    } else if (card->is_mmc) {
        type = "MMC";
        print_csd = true;
    } else {
        type = (card->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
        print_csd = true;
    }
    port->printf("Type: %s\r\n", type);
    if (card->max_freq_khz < 1000) {
        port->printf("Speed: %d kHz\r\n", card->max_freq_khz);
    } else {
        port->printf("Speed: %d MHz%s\r\n", card->max_freq_khz / 1000,
                card->is_ddr ? ", DDR" : "");
    }
    port->printf("Size: %lluMB\r\n", ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024));

    if (print_csd) {
        port->printf("CSD: ver=%d, sector_size=%d, capacity=%d read_bl_len=%d\r\n",
                (card->is_mmc ? card->csd.csd_ver : card->csd.csd_ver + 1),
                card->csd.sector_size, card->csd.capacity, card->csd.read_block_len);
        if (card->is_mmc) {
            port->printf("EXT CSD: bus_width=%d\r\n", (1 << card->log_bus_width));
        } else if (!card->is_sdio){ // make sure card is SD
            port->printf("SSR: bus_width=%d\r\n", (card->scr.bus_width ? 4 : 1));
        }
    }
    if (print_scr) {
        port->printf("SCR: sd_spec=%d, bus_width=%d\r\n", card->scr.sd_spec, card->scr.bus_width);
    }
}

/**
 * @brief Return the initialisation status of the SD Card. 
 * Request the initialisation status of the SD Card from the card status
 * global variable. 
 * @return true if the initialisation is still valid.
 * @return false if initialisation will need to be carried out.
 *****************************************************************************/
bool spiSdCardStatus()
{
    return (sdCardInfo.initDone);
}

/**
 * @brief Perform a 512 byte read of an SD card.
 * Peforms a 512 byte read of the SD card and throws away the data.
 * Function is used to see if we can read bytes from a SD card.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiSdCardDummyRead()
{
    uint8_t readBuffer[512];
    uint32_t sector = 0x100000/512;
    esp_err_t rc = sdmmc_read_sectors(sdCard, &readBuffer, sector, 1);
    if (rc != ESP_OK)
    {
        logger->printf("...sdmmc_read_sectors       bus=%d Failed  : 0x%04x\r\n", sdCard->host.slot, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...sdmmc_read_sectors       bus=%d Failed  : 0x%04x\r\n", sdCard->host.slot, rc);
#endif
        spiDecodeError(rc);
    } else {
        logger->printf("...sdmmc_read_sectors       bus=%d success : 0x%04x\r\n", sdCard->host.slot, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...sdmmc_read_sectors       bus=%d success : 0x%04x\r\n", sdCard->host.slot, rc);
#endif
    }
    return rc;
}

/**
 * @brief Test an SD card.
 * Reads an SD card and prints card information and raw data from the 
 * card as part of a user test to check that valid data is being read.
 * @param numSectors is the number of 512 byte sectors to read.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t testSdCard(uint16_t numSectors)
{
    // Print information about the card
    console->printf("\r\n");
    spiSdCardPrintInfo(console, sdCard);
    console->printf("\r\n");

    uint8_t readBuffer[512];
    bool bVerbose = true;
    uint32_t startSector = 0x100000/512;
    uint32_t sector;
    uint32_t raddr = startSector * 512;
    uint16_t addrInBlock = 0;
    esp_err_t rc = 0;

    // Loop one sector at a time rading 512 byes of data
    for (sector = startSector; sector < startSector + numSectors; sector++)
    {
        addrInBlock = 0;
        console->printf("*** sector = 0x%04x\r\n", sector);
        // Read the data from the specified sector
        rc = sdmmc_read_sectors(sdCard, &readBuffer, sector, 1);
        if (rc != ESP_OK){
            spiDecodeError(rc);
            return rc;
        }

        // Loop through each byte of the 512 byte data that was read and print
        // it to the console.
        while(addrInBlock < 512)
        {
            if (addrInBlock%16 == 0)
            {
                if (bVerbose) console->printf("%s%08x:", raddr==0?"":"\r\n", raddr);
            }
            if (bVerbose) console->printf(" %02x", readBuffer[addrInBlock]);
            raddr++;
            addrInBlock++;
        }
        console->printf("\r\n");
    }

    return rc;
}

/**
 * @brief Test to read the MCP23S17 IO Expander.  
 * Reads the GPIOA and GPIOBB registers of the MCP23S17 IO Expander devices.
 * The test function is used by the user to check that valid data is being read.
 * @param handle for the MCP23S17 devices.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t testIoExpander(spi_device_handle_t handle)
{
// MPC23S17 SPI IO expander address format
// 0 1 0 0 A2 A1 A0 R/W
#define IO_EXP1		(0x4C)	///< the one connected to the address bus
#define IO_EXP2		(0x48)	///< the one connected to the control and data bus
// The read/write bit is in bit position 0 with read = high, write = low
#define IO_EXP_RD   (0x01)

    // For chunks of 32bit data we can use the internal spi_transaction_t
    // tx and rx data buffers. Larger than 32bits need to be replaced with with 
    // a new buffer. 
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Set the tx/rx flags to use the small internal buffer.
    // Set the values for the tx buffer for the MCP23S17.
    // NOTE: we are not setting the HAEN bit here. Setting the HAEN bit
    // needs to be done to use individual device addressing as specified by
    // the A2-A0 address pins.
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 4*8;
    t.tx_data[0] = IO_EXP1 | IO_EXP_RD;
    t.tx_data[1] = 0x12;
    t.tx_data[2] = 0x0;
    t.tx_data[3] = 0x0;

    // Access the IO_EXP1 device
    // Send the message on the MOSI line and put MISO data in the rx buffer.
    esp_err_t rc = spi_device_transmit(handle, &t);
  
    console->printf("Transmit (Addr=0x%02x R/Wb=0x%x): 0x%02x 0x%02x 0x%02x 0x%02x \r\n", 
        (t.tx_data[0] & ~IO_EXP_RD), (t.tx_data[0] & IO_EXP_RD), 
        t.tx_data[0], t.tx_data[1], t.tx_data[2], t.tx_data[3]);
    // Only print receive data if this is a rerad
    if (t.tx_data[0] & IO_EXP_RD)
	{
        console->printf("Received (Addr=0x%02x R/Wb=0x%x): 0x%02x 0x%02x 0x%02x 0x%02x \r\n", 
            (t.tx_data[0] & ~IO_EXP_RD), (t.tx_data[0] & IO_EXP_RD), 
            t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
    }

    // Access the IO_EX2 device
    t.tx_data[0] = IO_EXP2 | IO_EXP_RD;
    rc = spi_device_transmit(handle, &t);
  
    console->printf("Transmit (Addr=0x%02x R/Wb=0x%x): 0x%02x 0x%02x 0x%02x 0x%02x \r\n", 
        (t.tx_data[0] & ~IO_EXP_RD), (t.tx_data[0] & IO_EXP_RD), 
        t.tx_data[0], t.tx_data[1], t.tx_data[2], t.tx_data[3]);
    // Only print receive data if this is a rerad
    if (t.tx_data[0] & IO_EXP_RD)
	{
        console->printf("Received (Addr=0x%02x R/Wb=0x%x): 0x%02x 0x%02x 0x%02x 0x%02x \r\n", 
            (t.tx_data[0] & ~IO_EXP_RD), (t.tx_data[0] & IO_EXP_RD), 
            t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
    }

    return rc;
}

/**
 * @brief Initialise one of the host SPI busses.
 * Initialises the user specified SPI host bus with the bus number and user 
 * specified pins. Sets the bus init status flag appropriately.
 * @param spiBus the spiBus class variable which will host the bus.
 * @param host the host bus number.
 * @param busConfig the configuartion of the bus
 * @param spiPinCfg the pin numbers to use.
 * @param statusBit 1-bit status bit in a byte to assign if successful.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiInitHost(SPIClass *spiBus, spi_host_device_t host, spi_bus_config_t *busConfig, wtmSpiBusPinCfg *spiPinCfg, uint8_t statusBit)
{
    // Start up SPI bus with the user specified pin settings
    spiBus->begin(spiPinCfg->clk, spiPinCfg->miso, spiPinCfg->mosi, spiPinCfg->cs0);

    // Initialise the SPI bus using auto DMA.
    esp_err_t rc = spi_bus_initialize(host, busConfig, SPI_DMA_CH_AUTO);

#if 0
    // Debug code to check pin assignment
    console->printf("   host = %d\r\n", host);
    console->printf("   clk  = %d\r\n", spiPinCfg->clk);
    console->printf("   miso = %d\r\n", spiPinCfg->miso);
    console->printf("   mosi = %d\r\n", spiPinCfg->mosi);
    console->printf("   cs0  = %d\r\n", spiPinCfg->cs0);
    console->printf("busConfig\r\n");
    console->printf("   sclk_io_num = %d\r\n", busConfig->sclk_io_num);
    console->printf("   miso_io_num = %d\r\n", busConfig->miso_io_num);
    console->printf("   mosi_io_num = %d\r\n", busConfig->mosi_io_num);
#endif

    if (rc != ESP_OK)
    {
        logger->printf("...spi_bus_initialize       bus=%d Failed  : 0x%04x\r\n", host, rc);
        console->printf("...spi_bus_initialize       bus=%d Failed  : 0x%04x\r\n", host, rc);
        spiDecodeError(rc);
    } else {
        spiStatus |= statusBit;
        logger->printf("...spi_bus_initialize       bus=%d success : 0x%04x\r\n", host, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...spi_bus_initialize       bus=%d success : 0x%04x\r\n", host, rc);
#endif    
    }

    return rc;
}

/**
 * @brief Free up one of the host SPI busses.
 * Frees up one of the previously initialised SPI host buses and sets the
 * bus init status flag appropriately.
 * @param spiBus the spiBus class variable to de-initialises.
 * @param host the host bus number.
 * @param statusBit 1-bit status bit in a byte to de-assign if successful.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiFreeHost(SPIClass *spiBus, spi_host_device_t host, uint8_t statusBit)
{
    // De-initialise the specified SPI bus.
    esp_err_t rc = spi_bus_free(spi_host_device_t(host));
    if (rc != ESP_OK)
    {
        logger->printf("...spi_bus_free             bus=%d Failed  : 0x%04x\r\n", host, rc);
        console->printf("...spi_bus_free             bus=%d Failed  : 0x%04x\r\n", host, rc);
        spiDecodeError(rc);
    } else {
        spiStatus &= ~statusBit;
        logger->printf("...spi_bus_free             bus=%d success : 0x%04x\r\n", host, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...spi_bus_free             bus=%d success : 0x%04x\r\n", host, rc);
#endif
    }

    // Shut down the SPI bus.
    spiBus->end();

    return rc;
}

/**
 * @brief Add a device to the SPI bus.
 * Adds a device to the SPI bus and assigns the Chip Select pin and frequency 
 * for that host. Sets the bus init status flag appropriately.
 * @param handle The handle that will be assigned for the device.
 * @param host The host bus to use this device on.
 * @param csPinNum The Chip Select pin number to use.
 * @param freqKhz The frequency in kHz for the device.
 * @param statusBit 1-bit status bit in a byte to assign if successful.
 * @param strDev A string describing the device to include during log prints. 
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiAddDeviceToBus(spi_device_handle_t *handle, spi_host_device_t host, int csPinNum, int freqKhz, uint8_t statusBit, const char * strDev)
{
    spi_device_interface_config_t devCfg={
        .command_bits     = 0,
        .address_bits     = 0,
        .dummy_bits       = 0,
        .mode             = 0,
        .duty_cycle_pos   = 128, // 50% duty cycle
        // Todo : optimize cs_ena_posttrans
        .cs_ena_posttrans = 1,   // Keep the CS low 1 cycles after transaction 
        .clock_speed_hz   = freqKhz * 1000,
        .spics_io_num     = csPinNum,
        .queue_size       = 3
    };

    esp_err_t rc = spi_bus_add_device(host, &devCfg, handle);
    if (rc != ESP_OK)
    {
        logger->printf("...spi_bus_add_device (%s) bus=%d Failed  : 0x%04x\r\n", strDev, host, rc);
        console->printf("...spi_bus_add_device (%s) bus=%d Failed  : 0x%04x\r\n", strDev, host, rc);
        spiDecodeError(rc);
    } else {
        spiStatus |= statusBit;
        logger->printf("...spi_bus_add_device (%s) bus=%d success : 0x%04x\r\n", strDev, host, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...spi_bus_add_device (%s) bus=%d success : 0x%04x\r\n", strDev, host, rc);
#endif
    }

    return rc;
}


/**
 * @brief Remove a device from the SPI bus.
 * Removes a device from the SPI busand sets the bus init status flag 
 * appropriately.
 * @param handle The handle that is assigned to the device.
 * @param statusBit 1-bit status bit in a byte to clear if successful.
 * @param strDev A string describing the device to remove during log prints. 
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiRemoveDevice(spi_device_handle_t handle, uint8_t statusBit, const char * strDev)
{
    esp_err_t rc = spi_bus_remove_device(handle);
    if (rc != ESP_OK)
    {
        logger->printf("...spi_bus_remove_device (%s)    Failed  : 0x%04x\r\n", strDev, rc);
        console->printf("...spi_bus_remove_device (%s)    Failed  : 0x%04x\r\n", strDev, rc);
        spiDecodeError(rc);
    } else {
        spiStatus &= ~statusBit;
        logger->printf("...spi_bus_remove_device (%s)    success : 0x%04x\r\n", strDev, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...spi_bus_remove_device (%s)    success : 0x%04x\r\n", strDev, rc);
#endif
    }

    return rc;
}

/**
 * @brief Add an SD card driver to the SPI bus.
 * Adds an SD card device driver to the SPI bus for an SD card and assigns the 
 * pins appropriatedly. Sets the bus init status flag appropriately.
 * @param devCfg the bus to use, CS pin, and other attribuates to use.
 * @param handle The handle that will be assigned for the device.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiAddSDCardDriver(sdspi_device_config_t *devCfg, sdspi_dev_handle_t *handle)
{
    esp_err_t rc = sdspi_host_init_device(devCfg, handle);
    if (rc != ESP_OK)
    {
        logger->printf("...sdspi_host_init_device   bus=%d Failed  : 0x%04x\r\n", devCfg->host_id, rc);
        console->printf("...sdspi_host_init_device   bus=%d Failed  : 0x%04x\r\n", devCfg->host_id, rc);
        spiDecodeError(rc);
    } else {
        spiStatus |= STATUS_INIT_SD;
        logger->printf("...sdspi_host_init_device   bus=%d success : 0x%04x\r\n", devCfg->host_id, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...sdspi_host_init_device   bus=%d success : 0x%04x\r\n", devCfg->host_id, rc);
#endif
    }
    return rc;
}

/**
 * @brief Initialise an SD card
 * Initialises an SD card device using a low level init call that does not
 * require knowing the file system. This allows the use of custom card
 * formatting such as used on the Z80-Retro project. Sets the bus init status 
 * flag appropriately.
 * @param spiPinCfgSdExp host configuration for the SD card
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiInitSdCard(wtmSpiBusPinCfg *spiPinCfgSdExp)
{
    sdmmc_host_t sdCfg = SDSPI_HOST_DEFAULT();
    sdCfg.max_freq_khz = spiPinCfgSdExp->freqkhz;
    sdCfg.slot = spiPinCfgSdExp->host;

    if ( !(spiStatus & STATUS_INIT_SD))
    {
        logger->printf("ERROR - spi bus not ready for sdmmc_card_init\r\n");
#ifdef WTM_DBG_SPI_INIT
        console->printf("ERROR - spi bus not ready for sdmmc_card_init\r\n");
#endif
        return ESP_ERR_INVALID_STATE;
    }
    sdCard = (sdmmc_card_t*)malloc(sizeof(sdmmc_card_t));

    esp_err_t rc = sdmmc_card_init(&sdCfg, sdCard);
    if (rc != ESP_OK)
    {
        logger->printf("...sdmmc_card_init           bus=%d Failed : 0x%04x\r\n", sdCfg.slot, rc);
        console->printf("...sdmmc_card_init           bus=%d Failed : 0x%04x\r\n", sdCfg.slot, rc);
        spiDecodeError(rc);
        return rc;
    } else {
        spiStatus |= STATUS_INIT_SD_CARD;
        sdCardInfo.initDone = true;
        logger->printf("...sdmmc_card_init          bus=%d success : 0x%04x\r\n", sdCfg.slot, rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...sdmmc_card_init          bus=%d success : 0x%04x\r\n", sdCfg.slot, rc);
#endif
        // Do a dummy read in order to check that the SD card is indeed in SPI mode.
        // rc = spiSdCardDummyRead();
    }

    return rc;
}


/**
 * @brief Remove an SD card device driver from the SPI bus.
 * Remove an SD card device driver from the SPI bus and sets the bus init 
 * status flag appropriately.
 * @param handle The handle that is assigned for the device.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t spiRemoveSDCardDriver(sdspi_dev_handle_t handle)
{
    esp_err_t rc = sdspi_host_remove_device(handle);
    if (rc != ESP_OK)
    {
        logger->printf("...sdspi_host_remove_device       Failed  : 0x%04x\r\n", rc);
        console->printf("...sdspi_host_remove_device       Failed  : 0x%04x\r\n", rc);
        spiDecodeError(rc);
    } else {
        spiStatus &= ~STATUS_INIT_SD;
        spiStatus &= ~STATUS_INIT_SD_CARD;
        logger->printf("...sdspi_host_remove_device       success : 0x%04x\r\n", rc);
#ifdef WTM_DBG_SPI_INIT
        console->printf("...sdspi_host_remove_device       success : 0x%04x\r\n", rc);
#endif
    }
    return rc;
}



/**
 * @brief Initialise the Main (on card) SPI bus.
 * 1. The first bus with the SD card and I/O expander and their chip select pins.
 * @return value is a combination of esp_err_t codes or'd teogether.
 * @return A non-zero value means there was an error reported.
 * @return ESP_OK value means everything is OK.
*****************************************************************************/
esp_err_t initSpiMain()
{
    spiPinCfgSdExp = {
        .host  = WTM_SPI_HOST_SD,
        .clk   = WTM_SPI_SD_CLK,
        .mosi  = WTM_SPI_SD_MOSI,
        .miso  = WTM_SPI_SD_MISO,
        .cs0   = WTM_SPI_SD_CS0,
        .cs1   = WTM_SPI_IO_EXP_CS1,
        .sdDet = WTM_SPI_SD_CD,
        .freqkhz = WTM_SPI_SD_FREQ,
    };

    // SPI bus configuration for the SD Card
    spi_bus_config_t busCfgSdCard = {
        .mosi_io_num = spiPinCfgSdExp.mosi,
        .miso_io_num = spiPinCfgSdExp.miso,
        .sclk_io_num = spiPinCfgSdExp.clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // Error codes will be returned from each of the initialisation functions.
    esp_err_t rcInit = 0;
    esp_err_t rcSd = 0;
    esp_err_t rcIoExp = 0;
    esp_err_t rcInitSdCard = 0;

    // Assign the CS pins as outputs = high per espressif documentation.
    pinMode(spiPinCfgSdExp.cs0, OUTPUT);
    digitalWrite(spiPinCfgSdExp.cs0, HIGH);
    pinMode(spiPinCfgSdExp.cs1, OUTPUT);
    digitalWrite(spiPinCfgSdExp.cs1, HIGH);

    // Set up the SD card detection variable that hold the plugged in state 
    // of the SD card and ultimately facilitate plug and unplug.
    sdCardDetectPinSetup();

    // Inititialise the on board SPI bus.
    rcInit = spiInitHost(&spiBusSd, hostSdAndIoExp, &busCfgSdCard, &spiPinCfgSdExp, STATUS_INIT_BUS_SD);

    // Inititialise the SD card driver
    sdspi_device_config_t sdDevConfig = SDSPI_DEVICE_CONFIG_DEFAULT();
    sdDevConfig.host_id = hostSdAndIoExp;
    sdDevConfig.gpio_cs = gpio_num_t(WTM_SPI_SD_CS0);
    // CD pin configuration is not needed as we have our own Card Detect algorithm
    // sdDevConfig.gpio_cd = gpio_num_t(WTM_SPI_SD_CD);
    if (rcInit == ESP_OK) rcSd = spiAddSDCardDriver(&sdDevConfig, &sdHandle);

    // Inititialise the MCP23S17 devices.
    // An SD card if present must be the first device that we talk to on the bus to ensure that the card
    // will switch from SDIO mode to SPI mode.
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/sdspi_share.html
    // This step will put the SD card into the SPI mode, which SHOULD be done before all other SPI 
    // communications on the same bus. Otherwise the card will stay in the SD mode, in which mode 
    // it may randomly respond to any SPI communications on the bus, even when its CS line is not addressed.
    if ((rcInit == ESP_OK) && (rcSd == ESP_OK) && (sdCardInfo.cardDetected)) rcInitSdCard = spiInitSdCard(&spiPinCfgSdExp);

    // Configure a CS for the two additional MCP23S17 devices on the bus.
    // MPC23S17 spec
    // CS Setup Time   = 50ns
    // CS Hold Time    = 50ns
    // CS Disable Time = 50ns
    // Data Setup Time = 10ns
    // Data Hold Time  = 10ns
    if (rcInit == ESP_OK) rcIoExp = spiAddDeviceToBus(&ioExpanderHandle, spiPinCfgSdExp.host, spiPinCfgSdExp.cs1, spiPinCfgSdExp.freqkhz, STATUS_INIT_IO_EXP, "Exp");

    // Copy the IO Expander handle for use in the Z80 flash programmer
    if ( (rcInit == ESP_OK) && (rcIoExp == ESP_OK) ) spiCopyHandleForExp(ioExpanderHandle);

    // A non-zero value means there was an error reported.
    // The code might not be a valid esp_err_t number.
    return rcInit | rcSd | rcInitSdCard | rcIoExp;
}


/**
 * @brief Initialise the Auxillary (off card) SPI bus.
 * 2. The second bus for the aux port and the CS pins it uses.
 * @return value is a combination of esp_err_t codes or'd teogether.
 * @return A non-zero value means there was an error reported.
 * @return ESP_OK value means everything is OK.
*****************************************************************************/
esp_err_t initSpiAux()
{
    spiPinCfgAux = {
        .host  = WTM_SPI_HOST_AUX,
        .clk   = WTM_SPI_AUX_CLK,
        .mosi  = WTM_SPI_AUX_MOSI,
        .miso  = WTM_SPI_AUX_MISO,
        .cs0   = WTM_SPI_AUX_CS0,
        .cs1   = 0,
        .sdDet = 0,
        .freqkhz = WTM_SPI_AUX_FREQ,
    };

    // Todo : Additional AUX pins to assign somewhere.
    //        Implement when there is an external device to add,
    // These are the specific pins which need code.
        // #define WTM_SPI_HOST_AUX    (SPI2_HOST)
        // #define WTM_AUX_RESET_N     (39)
        // #define WTM_AUX_DIAG        (14)
        // #define WTM_AUX_DONE        (3)

    // SPI bus configuration for the off-board Aux bus
    spi_bus_config_t busCfgAux = {
        .mosi_io_num = spiPinCfgAux.mosi,
        .miso_io_num = spiPinCfgAux.miso,
        .sclk_io_num = spiPinCfgAux.clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // Error codes will be returned from each of the initialisation functions.
    esp_err_t rcInitAux = 0;
    esp_err_t rcAux = 0;

    // Assign the CS pins as outputs = high per espressif documentation.
    pinMode(spiPinCfgAux.cs0, OUTPUT);
    digitalWrite(spiPinCfgAux.cs0, HIGH);

    // Inititialise the off board SPI bus Aux and assign a CS pin and frequency for the device being used.
    rcInitAux = spiInitHost(&spiBusAux, hostAux, &busCfgAux, &spiPinCfgAux, STATUS_INIT_BUS_AUX);
    if (rcInitAux == ESP_OK) rcAux = spiAddDeviceToBus(&auxHandle, spiPinCfgAux.host, spiPinCfgAux.cs0, spiPinCfgAux.freqkhz, STATUS_INIT_AUX_CS, "Aux");

    // Copy the IO Expander handle for use in the FPGA flash programmer
    if ( (rcInitAux == ESP_OK) && (rcAux == ESP_OK) ) spiCopyHandleForFpgaFlash(auxHandle);

    // A non-zero value means there was an error reported.
    // The code might not be a valid esp_err_t number.
    return rcAux | rcInitAux;
}


/**
 * @brief Initialise the board specific SPI buses.
 * Initialises the two SPI buses that are specific for the ESP WiFi & 
 * Programmer board.
 * 1. The first bus with the SD card and I/O expander and their chip select pins.
 * 2. The second bus for the aux port and the CS pin it uses.
 * @return value is a combination of esp_err_t codes or'd teogether.
 * @return A non-zero value means there was an error reported.
 * @return ESP_OK value means everything is OK.
*****************************************************************************/
esp_err_t initSpi()
{
    esp_err_t rcInitMain = 0;
    esp_err_t rcInitAux = 0;

    rcInitMain = initSpiMain();
    rcInitAux = initSpiAux();
    
    return rcInitMain | rcInitAux;
}

/**
 * @brief De-initialise the Main (on board) SPI bus.
 * 1. The first bus with the SD card and I/O expander.
 * @return value is a combination of esp_err_t codes or'd teogether.
 * @return A non-zero value means there was an error reported.
 * @return ESP_OK value means everything is OK.
*****************************************************************************/
esp_err_t deinitSpiMain()
{
    esp_err_t rcIoExp = 0;
    esp_err_t rcSd = 0;
    esp_err_t rcDeInit = 0;

    // Remove the MCP23S17 I/O Expander devices.
    if (spiStatus & STATUS_INIT_IO_EXP) rcIoExp = spiRemoveDevice(ioExpanderHandle, STATUS_INIT_IO_EXP, "Exp");

    // Remove the SD card driver
    if (spiStatus & STATUS_INIT_SD) rcSd = spiRemoveSDCardDriver(sdHandle);

    // Free the bus that was used by the SD card and IO expander
    if ( (rcIoExp == ESP_OK) && (rcSd == ESP_OK) && (spiStatus & STATUS_INIT_BUS_SD) ) 
        rcDeInit = spiFreeHost(&spiBusSd, hostSdAndIoExp, STATUS_INIT_BUS_SD);

    // A non-zero value means there was an error reported
    return rcDeInit | rcSd | rcIoExp;
}

/**
 * @brief De-initialise the Auxillary (off board) SPI bus.
 * 2. The second bus for the aux port.
 * @return value is a combination of esp_err_t codes or'd teogether.
 * @return A non-zero value means there was an error reported.
 * @return ESP_OK value means everything is OK.
*****************************************************************************/
esp_err_t deinitSpiAux()
{
    esp_err_t rcAux = 0;
    esp_err_t rcDeInitAux = 0;

    // Remoce the Aux port driver
    if (spiStatus & STATUS_INIT_AUX_CS)  rcAux = spiRemoveDevice(auxHandle, STATUS_INIT_AUX_CS, "Aux");

    // Free the bus that was used by the off-board AUX port
    if (spiStatus & STATUS_INIT_BUS_AUX)  
        rcDeInitAux = spiFreeHost(&spiBusAux, hostAux, STATUS_INIT_BUS_AUX); 

    pinMode(WTM_SPI_AUX_CLK, INPUT_PULLUP);
    pinMode(WTM_SPI_AUX_MOSI, INPUT_PULLUP);
    pinMode(WTM_SPI_AUX_MISO, INPUT_PULLUP);
    pinMode(WTM_SPI_AUX_CS0, INPUT_PULLUP);
    pinMode(WTM_SPI_AUX_CS1, INPUT_PULLUP);

    // A non-zero value means there was an error reported
    return rcAux | rcDeInitAux;
}

/**
 * @brief De-initialise the board specific SPI buses.
 * De-initialises the two SPI buses that are specific for the ESP WiFi & 
 * Programmer board.
 * 1. The first bus with the SD card and I/O expander.
 * 2. The second bus for the aux port.
 * @return value is a combination of esp_err_t codes or'd teogether.
 * @return A non-zero value means there was an error reported.
 * @return ESP_OK value means everything is OK.
*****************************************************************************/
esp_err_t deinitSpi()
{
    esp_err_t rcDeInitMain = 0;
    esp_err_t rcDeInitAux = 0;

    rcDeInitMain = deinitSpiMain();

    rcDeInitAux = deinitSpiAux();
    
    // A non-zero value means there was an error reported
    return rcDeInitMain | rcDeInitAux;
}

/**
 * @brief Handle SD card plug and unplug events
 * Handles events caused by the plug and unplug of the SD Card.
 * Reads the SD card global status variable that might have changed values.
 * Those changes can only have been made by the Interrupt Service Routine for 
 * the card detect pin. If a change is detected then print a log message and
 * initialise the SD card if appropriate.
 * *****************************************************************************/
// If the card present status has changed try just one time
// to initialise the SD card
void handleSdCardEvents()
{
    if (sdCardInfo.cardDetected)
    {
        // Card was detected as present. Check if this is a 
        // status change.
        if (!sdCardInfo.cardDetectPrev)
        {
            // De-bounce the pin and check the actual pin again.
            // Todo. Use a timer to lessen the delay.
            // Todo : improve so that init is redone after quick plud/unplu events.
            delay(500);
            uint8_t pinValue = digitalRead(sdCardInfo.detectPin);
            if (pinValue == 0x00)
            {
                logger->printf("...SD Card detected.\r\n");
#ifdef WTM_DBG_SPI_INIT
                console-printf("...SD Card detected.\r\n");
#endif
                esp_err_t rc = spiInitSdCard(&spiPinCfgSdExp);
                // Todo : remove?
                if (rc == ESP_OK) spiSetBusAvailable(true);
                sdCardInfo.cardDetectPrev = true;
            }
        }
    } else { 
        // Card was not detected as present
        if (sdCardInfo.cardDetectPrev)
        {
            // De-bounce the pin and check the actual pin again.
            // Todo. Use a timer to lessen the delay.
            delay(100);
            uint8_t pinValue = digitalRead(sdCardInfo.detectPin);
            if (pinValue == 0x01)
            {
                logger->printf("...SD Card removed.\r\n");
#ifdef WTM_DBG_SPI_INIT
                console-printf("...SD Card removed.\r\n");
#endif
                sdCardInfo.cardDetectPrev = false;
            }
        }
    }
}

// Todo : delete.
// #include "FS.h"
// #include "SD.h"
// #include "esp_vfs_fat.h"

/**
 * @brief Test to run a low level SPI test as part of board bring-up.  
 * 1. If an SD card is present it is initialised and a test read is 
 * performed on it. 
 * 2. A test is also carried out on the MCP23017 I/O Expander devices to 
 * read the contents of a single address on both devices. Note, this second
 * test assumes that the HAED bit in the IOCON register of the MCP23017 
 * has been set in order to know that the data is valid.
 * @param incSdCardTest set to true to test an SD card. False to skip.
 * @return esp_err_t error code. 
 *****************************************************************************/
esp_err_t lowLevelSpiTest(bool incSdCardTest)
{
    static bool initDone = false;
    // if (!initDone)
    // {
    //     esp_err_t rc = initSpi();
    //     initDone = true;
    //     if (rc != ESP_OK) return rc;
    // } else {
    //     sdspi_device_config_t sdDevConfig = SDSPI_DEVICE_CONFIG_DEFAULT();
    //     sdDevConfig.host_id = WTM_SPI_HOST_SD;
    //     sdDevConfig.gpio_cs = gpio_num_t(WTM_SPI_SD_CS0);
    //     // spiAddSDCardDriver(&sdDevConfig, &sdHandle);
    //     spiInitSdCard();
    // }

    // esp_vfs_fat_sdspi_mount();

    // if ( !(spiStatus & STATUS_INIT_SD_CARD) ) return ESP_ERR_INVALID_STATE;

    // SD card must be put into SPI mode before talking to other devices on the bus otherwise it
    // will stay in SD mode.
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/sdspi_share.html
    // lowLevelSDCardTest2();

    console->printf("\r\n");
    if (incSdCardTest) 
    {
        if (sdCardInfo.cardDetected)
        { 
            console-printf("...SD Card detected.\r\n");

            // WTM : Todo : Needs more investigating.
            // As the SD bus is shared to be safe we need to init the SD card each time
            // to get it back into SPI mode.
#if WTM_SPI_WORKAROUND
            spiInitSdCard(&spiPinCfgSdExp);
#endif

        } else {
            console-printf("...SD Card was not detected.\r\n");
        }

        if (sdCardInfo.initDone)
        {
            testSdCard(1);
        } else {
            console->printf("...ERROR SD card is not present or configured\r\n");
        }
    }
    console->printf("\r\n");
    // testIoExpander(ioExpanderHandle);
    console->printf("\r\n");

    //spiRemoveSDCardDriver(sdHandle);
    // deinitSpi();

    return ESP_OK;
}