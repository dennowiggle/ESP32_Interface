#include <stdio.h>
#include "SPI.h"
#include "driver/spi_master.h"
#include <LittleFS.h>

#include "defines.h"
#include "wtmSerialx.h"
#include "dbg.h"
#include "wtmFpga.h"
#include "wtmSpi.h"

extern wtmSerialX* console;
extern wtmSerialX* logger;

spi_device_handle_t spiHandleFpgaFlash;

/**
 * @brief Initialise the SPI bus to access the FPGA Flash.
 *****************************************************************************/
void initFpgaFlash()
{
    // First thing we need to do is toggle the FPGA reset line
    // and then hold it low so that we can write to the FLASH device
    // with no interference from the FPGA.
    pinMode(WTM_AUX_RESET_N, OUTPUT);
    digitalWrite(WTM_AUX_RESET_N, LOW);
    delay(1);
    digitalWrite(WTM_AUX_RESET_N, HIGH);
    delay(1);
    digitalWrite(WTM_AUX_RESET_N, LOW);
    delay(1);

    initSpiAux();
}

/**
 * @brief De-initialise the SPI bus to close access to the FPGA Flash.
 * This bus needs to be free'd up so that the FPGA can drive the SPI bus to 
 * load config from it's Flash memory.
 *****************************************************************************/
void deinitFpgaFlash()
{
    deinitSpiAux();
    delay(1);

    // Initiate FPGA configuration reset. Signal the FPGA reset line from LOW to
    // HIGH to signal time to load the configuration from FLASH (Config Reset).
    digitalWrite(WTM_AUX_RESET_N, HIGH);
}

/**
 * @brief Function to copy the spi handle from external source for use to 
 * communicate on the Aux Spi bus. 
 * The SPI driver will take care of assigning the handle for us.
 * @param handle is the spi handle for the FPGA FLASH device on the Aux spi bus 
 *****************************************************************************/
void spiCopyHandleForFpgaFlash(spi_device_handle_t handle)
{
    spiHandleFpgaFlash = handle;
}

/**
 * @brief Function to print the raw SPI TX and RX data
 * @param txdata is the data that was transmitted to the device
 * @param rxdata is the data that was received on the bus from the device
 * @param len is the number of bytes of valid data.
 * @param strCallingFunction is an aid to show which function called this.
*****************************************************************************/
#ifdef WTM_DBG_SPI_FPGA
void spiPrintRawData(uint8_t* txData, uint8_t* rxData, size_t len, const char * strCallingFunction, bool bPrintTx, bool bPrintRx)
{
	if (len > 0 && bPrintTx)
	{
		console->printf("%s: Transmit (CMD=0x%02x):", strCallingFunction, txData[0]);
		for (int index = 0; index<len; index++)
		{
			console->printf(" %02x", txData[index]);
		}
		console->printf("\r\n");
    }

	if (len > 0 && bPrintRx)
    {
        console->printf("%s: Receive          ():", strCallingFunction);
        for (int index = 0; index<len; index++)
        {
            console->printf(" %02x", rxData[index]);
        }
        console->printf("\r\n");
	}
}
#endif /* WTM_DBG_SPI_FPGA */

/**
 * @brief Function to Read/Write the raw SPI RX and TX data to the FPGA or Flash device
 * @param data is the data to transmit to the slave device 
 * @param data return value is then copied with the RX data received back from the slave device
 * @param len is the number of bytes of valid data.
 * @return error status code.
 *****************************************************************************/
esp_err_t spiReadWrite(spi_device_handle_t handle, uint8_t* data, const int len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length =len * 8;
    t.tx_buffer = data;
    t.rx_buffer = data;

#ifdef WTM_DBG_SPI_FPGA
    spiPrintRawData(data, data, len, __FUNCTION__, true, false);
#endif

    esp_err_t rc = spi_device_transmit(handle, &t);

	if(rc != ESP_OK)
	{
        DBG("spiReadWrite() failed %d\r\n", rc);
		return rc;
	}

#ifdef WTM_DBG_SPI_FPGA
    spiPrintRawData(data, data, len, __FUNCTION__, false, true);
#endif

    return ESP_OK;
}


/**
 * @brief Function to read the FLASH JEDEC ID data and print to console
 * Use this to confirm the SPI bus and FLASH are operating okay.
 * @param txdata is the data that was transmitted to the device
 * @param rxdata is the data that was received on the bus from the device
 * @param len is the number of bytes of valid data.
 * @return true if FLASH is recognised.
*****************************************************************************/
bool readFpgaFlashId()
{
    // Define the JEDEC ID command 0x9F to get the manufacturer and device information
    // W25Q32 Datasheet pdf page 22 : Instruction Set Table 1 (Standard SPI Instructions)
    uint8_t cmd = 0x9F;
    const uint8_t bufLen = 4;
    bool goodFlashId = false;

    // Declare the Tx/Rx Buffer
    uint8_t* buffer = new uint8_t[bufLen];
    memset(buffer, 0, bufLen);

    buffer[0] = cmd;
    esp_err_t rc = spiReadWrite(spiHandleFpgaFlash, buffer, bufLen);

    console->printf("FPGA Flash Manufacturer Id = 0x%02x\r\n", buffer[1]);
    console->printf("FPGA Flash ID              = 0x%04x\r\n", buffer[2] << 8 | buffer[3]);

    if ( (rc == ESP_OK) &&  (
        // Winbond W25Q32
         (buffer[1] == 0xef) && (buffer[2] == 0x40) && (buffer[3] == 0x16)
         // Add other devices here
         ) )
    {
        console->printf("FLASH ID is good\r\n");
        goodFlashId = true;
    } else {
        console->printf("FLASH ID is not recognised or the SPI bus is not functioning correctly.\r\n");
        goodFlashId = false;
    }

    delete[] buffer;

    return goodFlashId;
}

/**
 * @brief Function to wait until the FLASH is no longer busy.
 * Reads the Status Register-1 busy bit and loops until the value is 0.
 *****************************************************************************/
esp_err_t waitForCompleteFpgaFlash()
{
    // Define the Read Status Register-1 command 0x05
    // W25Q32 Datasheet pdf page 22 : Instruction Set Table 1 (Standard SPI Instructions)
    //                  pdf page 13 : 6. STATUS AND CONFIGURATION REGISTERS
    uint8_t cmd = 0x05;
    const uint8_t bufLen = 2;
    bool transactionComplete = false;
    esp_err_t rc;

    // Declare the Tx/Rx Buffer
    uint8_t* buffer = new uint8_t[bufLen];

    while (transactionComplete == false)
    {
        buffer[0] = cmd;
        buffer[1] = 0;
        rc = spiReadWrite(spiHandleFpgaFlash, buffer, bufLen);
        if ((buffer[1] & 0x01) == 0)
        {
            transactionComplete = true;
        }
    }

    delete[] buffer;

    return rc;
}

/**
 * @brief Function to read a block of data from the FPGA FLASH.
 * Reads the Status Register-1 busy bit and loops until the value is 0.
 * @param flashAddr is the start address of block to read
 * @param data is a buffer to hold the data that will be read
 * @param dataLen is the number of bytes of data to read.
 *****************************************************************************/
esp_err_t readFpgaFlash(uint32_t flashAddr, uint8_t* data, const uint16_t dataLen)
{

    // Define the read command 0x03
    // W25Q32 Datasheet pdf page 22 : Instruction Set Table 1 (Standard SPI Instructions)
    uint8_t cmd = 0x03;
    const uint16_t bufLen = dataLen + 4;

    // Declare the Tx/Rx Buffer
    uint8_t* buffer = new uint8_t[bufLen];
    memset(buffer, 0, bufLen);

    buffer[0] = cmd;
    buffer[1] = (flashAddr >> 16) & 0xFF;
    buffer[2] = (flashAddr >>  8) & 0xFF;
    buffer[3] = flashAddr & 0xFF;

    esp_err_t rc = spiReadWrite(spiHandleFpgaFlash, buffer, bufLen);
    memcpy(data, &buffer[bufLen - dataLen], dataLen);

    delete[] buffer;

#ifdef WTM_DBG_SPI_FPGA_2
    console->printf("\r\n....Read flashAddr = %02x %02x %02x\r\n",  ((flashAddr >> 16) & 0xFF),  ((flashAddr >>  8) & 0xFF), (flashAddr & 0xFF));
#endif

    return rc;
}

/**
 * @brief Function to enable writing to the FPGA FLASH.
 *****************************************************************************/
esp_err_t writeEnableFpgaFlash()
{
    // Define the write enable command 0x06
    // W25Q32 Datasheet pdf page 22 : Instruction Set Table 1 (Standard SPI Instructions)
    uint8_t cmd = 0x06;
    esp_err_t rc = spiReadWrite(spiHandleFpgaFlash, &cmd, 1);

    return rc;
}

/**
 * @brief Function to disable writing to the FPGA FLASH.
 *****************************************************************************/
esp_err_t writeDisableFpgaFlash()
{
    // Define the write disable command 0x04
    // W25Q32 Datasheet pdf page 22 : Instruction Set Table 1 (Standard SPI Instructions)
    uint8_t cmd = 0x04;
    esp_err_t rc = spiReadWrite(spiHandleFpgaFlash, &cmd, 1);

    return rc;
}


/**
 * @brief Function to erase a 32k block of the FPGA FLASH.
 * User must enable writing before calling this function, and call the wait for
 * complete function after.
 * @param flashAddr is the start address of sector to erase
 *****************************************************************************/
esp_err_t eraseFpgaFlash32k(uint32_t flashAddr)
{
    // Define the erase 32kB command 0x52
    // W25Q32 Datasheet pdf page 22 : Instruction Set Table 1 (Standard SPI Instructions)
    uint8_t cmd = 0x52;
    const uint16_t bufLen = 4;

    // Declare the Tx/Rx Buffer
    uint8_t* buffer = new uint8_t[bufLen];
    buffer[0] = cmd;
    buffer[1] = (flashAddr >> 16) & 0xFF;
    buffer[2] = (flashAddr >>  8) & 0xFF;
    buffer[3] = flashAddr & 0xFF;
    esp_err_t rc = spiReadWrite(spiHandleFpgaFlash, buffer, bufLen);
    waitForCompleteFpgaFlash();
    delete[] buffer;

#ifdef WTM_DBG_SPI_FPGA_2
    console->printf("\r\n....Erased flashAddr = %02x %02x %02x\r\n",  ((flashAddr >> 16) & 0xFF),  ((flashAddr >>  8) & 0xFF), (flashAddr & 0xFF));
#endif

    return rc;
}

/**
 * @brief Function to erase an image sized area of the FPGA FLASH in 32kB chunks.
 * @param flashAddr is the start address of sector to erase. Needs to be
 * aligned to a 32k boundary.
 * @param flashSize is the size of the flash for checks to ensure we do not 
 * write past the end of flash
 * @param size is the size of memory to erase. It will be rounded up to
 * the nearest 32K block boundary.
 *****************************************************************************/
int eraseFpgaFlash(uint64_t flashAddr, uint64_t flashSize, uint64_t size)
{
    uint64_t eraseBlockSize = 32 * 1024;
    uint64_t startAddr = flashAddr & 0xFFFF8000;
    if (startAddr != flashAddr)
    {
        console->printf("The start erase address does not align to a 32kB boundary\r\n");
        return -1;
    }

    // Calculate the number of 32kB block erase cycles that are needed
    uint32_t num32kBlocks = size / eraseBlockSize;
    if ( (eraseBlockSize * num32kBlocks) <  size)
    {
        num32kBlocks += 1;
    }
    console->printf("Erasing %d x 32K blocks.\r\n", num32kBlocks);

    // Check that this doesn't go past the end of the flash
    uint64_t endAddr = startAddr + (eraseBlockSize * num32kBlocks);
    if ( endAddr > flashSize)
    {
        console->printf("The end erase address goes past the end of Flash\r\n");
        return -1;
    }
    for (uint64_t eraseAddr = startAddr; eraseAddr < endAddr; eraseAddr += eraseBlockSize)
    {
        console->printf("%3d%% 0x%08llx erasing: \r", 100*(eraseAddr - startAddr)/(endAddr - startAddr), eraseAddr);
        // Todo : check error codes.
        writeEnableFpgaFlash();
        eraseFpgaFlash32k(eraseAddr);
        waitForCompleteFpgaFlash();
    }
    console->printf("%3d%% 0x%08llx erasing: \r\n", 100, endAddr - 1);

    return 0;
}

/**
 * @brief Function to program a block of data into the FPGA FLASH.
 * @param flashAddr is the start address of block to write.
 * @param data is a buffer that holds the data to write.
 * @param dataLen is the number of bytes of data to write (max 256).
 *****************************************************************************/
void writeFpgaFlashBlock(uint64_t flashAddr, uint8_t* data, uint16_t dataLen)
{
    // Define the write page proogram command 0x02. Max bytes = 256.
    // W25Q32 Datasheet pdf page 22 : Instruction Set Table 1 (Standard SPI Instructions)
    // W25Q32 Datasheet pdf page 35 : 7.2.13 Page Program (02h) 
    uint8_t cmd = 0x02;
    const uint16_t bufLen = dataLen + 4;

    // Declare the Tx/Rx Buffer
    uint8_t* buffer = new uint8_t[bufLen];
    memset(buffer, 0, bufLen);

    buffer[0] = cmd;
    buffer[1] = (flashAddr >> 16) & 0xFF;
    buffer[2] = (flashAddr >>  8) & 0xFF;
    buffer[3] = flashAddr & 0xFF;

    memcpy(&buffer[bufLen - dataLen], data, dataLen);
    spiReadWrite(spiHandleFpgaFlash, buffer, bufLen);

    delete[] buffer;
}

/**
 * @brief Function to program a block of data into the FPGA FLASH.
 * @param flashAddr is the start address of block to write.
 * @param data is a buffer that holds the data to write.
 * @param dataLen is the number of bytes of data to write (max 256).
 *****************************************************************************/
void writeFpgaFlash(uint64_t flashAddr, uint8_t* data, uint16_t dataLen)
{
    writeEnableFpgaFlash();
    writeFpgaFlashBlock(flashAddr, data, dataLen);
    waitForCompleteFpgaFlash();
    writeDisableFpgaFlash();
}

/**
 * @brief Function to test reading, erasing, writing the FPGA FLASH.
 *****************************************************************************/
void testFpgaFlash()
{
    console->printf("Test 1 ....... Read FPGA FLASH ID\r\n");
    readFpgaFlashId();
    console->printf("\r\n");

    uint8_t buffer[PAGE_BLOCK_SIZE];
    uint16_t nbytes = PAGE_BLOCK_SIZE;

    uint32_t startAddr = 0;
    uint32_t readAddr = 0;
    uint32_t writeAddr = 0;
    uint16_t bufIndex = 0;

    console->printf("Test 2 ....... Enable writing.\r\n");
    writeEnableFpgaFlash();
    console->printf("\r\n");

    console->printf("Test 3 ....... Erase block starting at address = 0x%06x.\r\n", startAddr);
    eraseFpgaFlash32k(startAddr);
    console->printf("\r\n");

    console->printf("Test 4 ....... Wait for complete.\r\n");
    waitForCompleteFpgaFlash();
    console->printf("\r\n");

    console->printf("Test 5 ....... Read 0x%04x bytes starting at address = 0x%06x.\r\n", nbytes, startAddr);
    readFpgaFlash(startAddr, buffer, nbytes);
    console->printf("\r\n");

    console->printf("Test 6 ....... Dump the data read.\r\n");
    // Loop through nbytes worth of addresses
	while( (readAddr - startAddr) < nbytes)
	{
        if (readAddr%16 == 0)
        {
            console->printf("%s%04x:", readAddr==0?"":"\r\n", readAddr);
        }
        console->printf(" %02x", buffer[bufIndex]);
		readAddr++;
        bufIndex++;
	}
    console->printf("\r\n\n");

    console->printf("Test 7 ....... Erase size = 655690 starting at address = 0x%06x.\r\n", startAddr);
    eraseFpgaFlash(startAddr, 0x400000, 655690);
    console->printf("\r\n");
 
    console->printf("Test 8 ....... Enable writing.\r\n");
    writeEnableFpgaFlash();
    console->printf("\r\n");

    for (writeAddr = startAddr; writeAddr < (startAddr + nbytes); writeAddr++)
    {
        buffer[writeAddr] = 0xAA;
    }

    console->printf("Test 9 ....... write 0x%04x bytes starting at address = 0x%06x.\r\n", nbytes, startAddr);
    writeFpgaFlashBlock(startAddr, buffer, nbytes);
    console->printf("\r\n");
    console->printf("Test 10 ....... Wait for complete.\r\n");
    waitForCompleteFpgaFlash();
    console->printf("\r\n");

    console->printf("Test 11 ....... Disable writing.\r\n");
    writeDisableFpgaFlash();
    console->printf("\r\n");

    bufIndex = 0;

    console->printf("Test 12 ....... Read 0x%04x bytes starting at address = 0x%06x.\r\n", nbytes, startAddr);
    readFpgaFlash(startAddr, buffer, nbytes);
    console->printf("\r\n");

    console->printf("Test 13 ....... Dump the data read.\r\n");
    readAddr = startAddr;
    bufIndex = 0;
    // Loop through nbytes worth of addresses
	while( (readAddr - startAddr) < nbytes)
	{
        if (readAddr%16 == 0)
        {
            console->printf("%s%04x:", readAddr==0?"":"\r\n", readAddr);
        }
        console->printf(" %02x", buffer[bufIndex]);
		readAddr++;
        bufIndex++;
	}
    console->printf("\r\n");

}
