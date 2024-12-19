 /*************************************************************************
 * 
 *    Code & ESP Programmer Board by Denno Wiggle aka WTM
 * 
 *    This is code to program the Z80 FLASH memory.
 * 
 *    Two programming methods are available:
 *    1. Program using Xmodem on a serial or telnet terminal.
 *    2. Program using the web interface.
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
#include <Arduino.h>
#include <stdio.h>
#include "defines.h"
#include "flash.h"
#include "wtmFpga.h"
#include "wtmProgrammer.h"
#include "xmodem.h"
#include "dbg.h" 
#include "wtmSerialx.h"


extern wtmSerialX* xmodemUart;
extern wtmSerialX* console;


/**
 * @brief Get the flash type that is currently set for programming. Options are
 * 1. Z80_FLASH (default)
 * 2. FPGA_FLASH
*****************************************************************************/
uint8_t wtmProgrammer::getFlashType()
{
    return _flashType;
}

/**
 * @brief Set the flash type to be used for programming. Also set the default
 * filename for programming the flash device. Options are
 * 1. Z80_FLASH (default)
 * 2. FPGA_FLASH
*****************************************************************************/
void wtmProgrammer::setFlashType(wtmFlashType setting)
{
    _flashType = setting;
    if (setting == Z80_FLASH)
    {
        _flashFileName = "/z80retro.bin";
        _flashSize = Z80_FLASH_SIZE;
    } else {
        _flashFileName = "/FLEADiP.bin";
        _flashSize = FPGA_FLASH_SIZE;
    }
}

/**
 * @brief Request and take over the Z80 bus for our use.
*****************************************************************************/
void wtmProgrammer::selectZ80Bus()
{
    int      iic = 0;
    mcp23017_init(iic);
}

/**
 * @brief Request and take over the programmer bus for our use.
 * 1. If selected the Z80 bus is requested and taken control off.
 * 2. If selected FPGA spi bus set up.
 * Note: setFlashType() should be called before calling this function.
*****************************************************************************/
void wtmProgrammer::selectBus()
{
    if (_flashType == Z80_FLASH)
    {
        selectZ80Bus();
    } else {
        initFpgaFlash();
    }
}

/**
 * @brief Return the Z80 bus control to the Z80 CPU and toggle reset. 
*****************************************************************************/
void wtmProgrammer::releaseZ80Bus()
{
    int      iic = 0;
    bus_release(iic);
}

/**
 * @brief Return the bus control to the rightful owner. 
 * 1. If selected the Z80 bus reset line is asserted
 * 2. If selected FPGA spi bus is released.
 * reset line.
*****************************************************************************/
void wtmProgrammer::releaseBus()
{
    if (_flashType == Z80_FLASH)
    {
        releaseZ80Bus();
    } else {
        deinitFpgaFlash();
    }
}

/**
 * @brief Reset the Z80 CPU by initialising the bus for 1 second and then
 * releasing so that reset is applied. Shorter times work too, 
 * but 1 second is noticeable by the user to know that reset has taken place.
*****************************************************************************/
void wtmProgrammer::resetZ80()
{
    // Request and take over the Z80 bus.
    selectZ80Bus();
    // Add a one second delay so the user can see the reset happen.
    delay(1000);
    // Toggle reset and return the bus to the Z80 CPU.
	releaseZ80Bus();
}

/**
 * @brief Read the Z80 FLASH Id
 * @return the FLASH ID codes for manucaturer and device
*****************************************************************************/
bool wtmProgrammer::readZ80FlashId()
{
    int     iic = 0;
    bool    goodFlashId;
	
	uint16_t flashId = flash_read_product_id(iic);
    console->printf("Z80 Flash Manufacturer Id = 0x%02x\r\n", (flashId & 0xff00) >> 8);
    console->printf("Z80 Flash ID              = 0x%02x\r\n", flashId & 0xff);
    if (flashId != 0xbfb5 && flashId != 0xbfb6 && flashId != 0xbfb7)
	{
		console->printf("Invalid FLASH signature: 0x%04x != 0xbfb5\r\n", flashId);
        console->flush();
		goodFlashId = false;
	} else {
        console->printf("FLASH ID is good\r\n");
        goodFlashId = true;
    }

    // As we are printing to the serial port with printf's in flash.cpp which is slow 
    // we need to wait until the print has completed before moving on.
    fflush(stdout);

    return goodFlashId;
}

/**
 * @brief Read the FLASH Id
 * @return the FLASH ID codes for manucaturer and device
*****************************************************************************/
bool wtmProgrammer::readFlashId()
{
    bool rc= false;
    if (_flashType == Z80_FLASH)
    {
        rc = readZ80FlashId();
    } else {
        rc = readFpgaFlashId();
    }

    return rc;
}

/**
 * @brief Erase all bytes in the Z80 FLASH.
 * @return 0 on success
 * @return -1 on failure
*****************************************************************************/
int wtmProgrammer::eraseZ80Flash()
{	
    int     iic = 0;
    int     rc = 0;

    console->printf("Erasing entire chip ....");
	rc = flash_chip_erase(iic);
    if(rc != 0)
	{
        DBG("Erase failed, rc = %d\r\n", rc);
		return -1;
	}
    console->printf(" done\r\n");
	return 0;
}

/**
 * @brief Erase the necessary space in the Flash
 * 1. If selected the Z80 Flash will be erased.
 * 2. If FPGA flash selected then only space required will be erased
 *    to the nearest 32kByte boundary.
 * @return 0 on success
 * @return -1 on failure
*****************************************************************************/
int wtmProgrammer::eraseFlash()
{	
    int rc;

    if (_flashType == Z80_FLASH)
    {
        rc = eraseZ80Flash();
    } else {
        uint64_t eraseSize;
        if (_programFile)
        {
            if (_programFile.size() > 0)
            {
                eraseSize = std::min(uint64_t(_programFile.size()), uint64_t(FPGA_FLASH_SIZE) );
            } else {
                eraseSize = FPGA_IMAGE_SIZE;
            }
        } else {
            eraseSize = FPGA_IMAGE_SIZE;
        }
        rc = eraseFpgaFlash(0, FPGA_FLASH_SIZE, eraseSize);

        // Todo : Check return code above before verifying
        rc = verifyFlashBlank(0, eraseSize);
    }

    return rc;
}
int wtmProgrammer::verifyFlashBlank(uint64_t startAddr, uint64_t endAddr)
{
    uint8_t blankBuffer[BLOCK_SIZE] = {0xff};
    uint16_t verifyBlockSize = BLOCK_SIZE;
    int rc = 0;

    for (int i = 0; i < verifyBlockSize; i++)
    {
        blankBuffer[i] = 0xff;
    }

    bool boolFirstLoop = true;
    for (uint64_t verifyAddr = startAddr; verifyAddr < endAddr; verifyAddr += verifyBlockSize)
    {
        // Limit the prints to 32K blocks to save printing time
        if ( (boolFirstLoop) || ( (verifyAddr % (32 * 1024) ) == 0) )
        {
            console->printf("%3d%% 0x%08llx verifying erased: \r", 100*(verifyAddr - startAddr)/(endAddr - startAddr), verifyAddr);
            boolFirstLoop = false;
        }

        if (_flashType == Z80_FLASH)
        {
            readOneFlashBlock(verifyAddr, verifyBlockSize, _verboseProgramming);
        } else {
            readFpgaFlash(verifyAddr, _readBuffer, verifyBlockSize);
        }
        // For long tasks we need to let other task get a chance to execute
        yield();

        if (memcmp(blankBuffer, _readBuffer, verifyBlockSize) != 0)
        {
            // There was an error verifying data
            console->printf("\r\nError verifying erase at block address 0x%04x\r\n", verifyAddr);
            // Todo: determine the right reurn value
            rc = -1;

        } else {
        }
    }
    console->printf("%3d%% 0x%08llx verifying erased: \r\n", 100, endAddr - 1);

    return rc;
}

/**
 * @brief Program one block of data to Z80 Flash memory
 * User prints are enabled if _verboseProgramming is set to true.
 *    This option shows the data being programmed in a neat table.
 * @param startAddr is the address to use to start programming the data.
 * @param buffer contains the byte data to program into memory.
 * @param nBytes is the number of bytes to program.
*****************************************************************************/
void wtmProgrammer::writeZ80FlashBlock(uint16_t startAddr, uint8_t* buffer, uint16_t nBytes)
{
    int      iic = 0;
    uint16_t writeAddr = startAddr;
    uint16_t byteNum = 0;
    bool     boolEndFlash = false;
    
    // Loop through nbytes worth of addresses or until
    // we reach the end of the Flash
	while( ( (writeAddr - startAddr) < nBytes) && !boolEndFlash )
	{
        byte wbyte = buffer[byteNum++];
        // Program one byte at a time (as specified in the Flash data sheet)
        // Function call uses a very specific sequence of byte writes to unlock writing
        // mode and then write one byte of data. This must be done for each byte
		flash_program_byte(iic, writeAddr, wbyte);

        if (_verboseProgramming) 
        {
            if (writeAddr%16 == 0)
            {
                console->printf("%s%04x:", writeAddr==0?"":"\r\n", writeAddr);
            }
            console->printf(" %02x", wbyte);
        }
		writeAddr++;
        // Check to see if we have reached the end of the Flash
        // or we have rolled over to 0 on the address bus
        if ( (writeAddr == _flashSize) || (writeAddr == 0) )
        {
            boolEndFlash = true;
        }
        yield();
	}
    if (_verboseProgramming) {
        console->printf("\r\n\n");
    }
}

/**
 * @brief Read one block of data from the FLASH memory into the private 
 * read buffer _readBuffer. This is used to verify data programmed.
 * @param addr is the address to use to start reading the data.
 * @param nBytes is the number of bytes to read.
 * @param bVerbose determines whether or not to print the data to console.
 *        if bVerbose is enabled the data is shown in a neat table.
*****************************************************************************/
void wtmProgrammer::readOneFlashBlock(uint32_t addr, uint16_t nbytes = BLOCK_SIZE, bool bVerbose = true)
{
    int      iic = 0;
    uint32_t raddr = addr;
    uint16_t bufIndex = 0;
    bool     boolEndFlash = false;

    if (bVerbose) console->printf("\r\n");

    // Loop through nbytes worth of addresses or until
    // we reach the end of the Flash
	while( ( (raddr - addr) < nbytes) && !boolEndFlash )
	{
        if (_flashType == Z80_FLASH)
        {
            _readBuffer[bufIndex] = bus_read_cycle(iic, raddr);
        } else {
            readFpgaFlash(raddr, &_readBuffer[bufIndex], 1);
        }
    
        if (raddr%16 == 0)
        {
            if (bVerbose) console->printf("%s%04x:", raddr==0?"":"\r\n", raddr);
        }
        if (bVerbose) console->printf(" %02x", _readBuffer[bufIndex]);
		raddr++;
        bufIndex++;
        // Check to see if we have reached the end of the Flash
        // or we roll over to 0 on the address bus
        if ( (raddr == _flashSize) || (raddr == 0) )
        {
            boolEndFlash = true;
        }
        yield();
	}
    if (bVerbose) console->printf("\r\n");
}

/**
 * @brief Program data in the file buffer to the FLASH memory.
 * Programming is carried out in blocks with the following steps.
 * 1. Read a block from file into the file buffer
 * 2. Program that block into flash memory
 * 3. Read back the block of data in Flash that we just programmed
 * 4. Verify the the read-back block of Flash with the original block of 
 * file data. 
 * This should have a retry mechanism on failure. Deemed not necessary 
 * for a hobby project. 
 * @param addr is the address to use to start reading the data.
 * @param nBytes is the number of bytes to read.
 * @param bVerbose determines whether or not to print the data to console.
 *        if bVerbose is enabled the data is shown in a neat table.
 * @return true if programming is successful
 * @return false if error detected in programming the data
*****************************************************************************/
bool wtmProgrammer::programBlock(uint8_t* fileBuffer, int nBytes)
{
    // Update the percentage complete / address counter to console
    if (_addr % (BLOCK_SIZE) == 0) {
        if (_flashType == Z80_FLASH)
        {
            console->printf("%3d%% 0x%06x writing: \r", 100*_addr/int(_programFile.size()), _addr);
        } else {
            // For the FPGA FLASH print an update at a longer interval
            if (_addr % (32 * 1024) == 0) 
            {
                console->printf("%3d%% 0x%06x writing: \r", 100*_addr/int(_programFile.size()), _addr);
            }
        }
    }
    
    // Limit the bytes to program to the block size.
    int numBytes = std::min(BLOCK_SIZE, nBytes);

    // Write a block of data
    if (_verboseProgramming) console->printf("Writing block of data\r\n");
    if (_flashType == Z80_FLASH)
    {
        writeZ80FlashBlock(_addr, fileBuffer, numBytes);
    } else {
        writeFpgaFlash(_addr, fileBuffer, numBytes);
    }
    // For long tasks we need to let other task get a chance to execute
    yield();

    // Read back the block of data from Flash
    if (_verboseProgramming) console->printf("Reading block of data\r\n");
    if (_flashType == Z80_FLASH)
    {
        readOneFlashBlock(_addr, numBytes, _verboseProgramming);
    } else {
        readFpgaFlash(_addr, _readBuffer, numBytes);
    }
    // For long tasks we need to let other task get a chance to execute
    yield();

    // Verify the block of data read from Flash with the original copy from file
    bool success = false;
    if (memcmp(fileBuffer, _readBuffer, numBytes) != 0)
    {
        // There was an error verifying data
        console->printf("Error verifying data at block address 0x%04x\r\n", _addr);
        // Show where the error occurred by comparing the two blocks
        // byte for byte.
        console->printf("    Compare File:Flash \r\n");
        for (int i = 0; i < numBytes; i++) {
            if (i%8 == 0)
            {
                console->printf("%s    %04x:", i==0?"":"\r\n", _addr + i);
            }
            console->printf(" %02x:%02x ", fileBuffer[i], _readBuffer[i]);
        }
        console->printf("\r\n");
        // Return false as we have had an error
        success = false;

        // Todo : In the future we could add a retry mechanism on a verify fail.
        // However, if we did that we would need to implement block by block erase
        // for the Z80 flash and reread the data from file. 
        // Probably not worth the effort.

        _addr += numBytes;

    } else {
        success = true;
        _addr += numBytes;

        if (_verboseProgramming) console->printf("Block 0x%04x verifies OK\r\n", _addr);

        // Update the percentage complete / address counter if programming is complete
        if ( (_addr) >= _programFile.size() ) 
            console->printf("%3d%% 0x%06x writing: \r", 100*_addr/int(_programFile.size()), _addr);

        // Check if we have reached the end of the flash
        if (_addr >= _flashSize) {
            console->printf("\r\n");
            console->printf("Finished Programming at the end of Flash\r\n");
            _boolEndFlash = true;
        }
    }

    return success;
}

/**
 * @brief Program the flash firmware using the console
 * There could be an option to just set beginProgramming() but 
 * then the console menu would show before programming is complete.
 *****************************************************************************/
void wtmProgrammer::programFlashFirmware()
{
    beginProgramming((char *)"");

    while (_programmingActive)
    {
        updateProgramming();
    }

}

/**
 * @brief Set the serial or telnet port to be used for xmodem.
 * @param portId is the port to use for programming with xmodem.
*****************************************************************************/
void wtmProgrammer::selectXmPort(uint8_t portId)
{
    xmodemUart->disablePort(0xFF);
    xmodemUart->enablePort(portId);
}

/**
 * @brief If programming via web rather than console then set this state 
 * which is used to prevent the programming task from blocking other tasks.
*****************************************************************************/
void wtmProgrammer::setProgrammingFromWeb(bool state)
{
    _programmingFromWeb = state;
}

/**
 * @brief Read and save 64K from the currently selected FLASH to file with the default
 * filename for that FLASH.
*****************************************************************************/
void wtmProgrammer::save64kFlashToFile()
{
    // Set the size to 64K
    int sizeToRead = 64 * 1024;
    File fsSaveFile;
    // Start at address 0
    uint32_t address = 0;

    // Open a file for writing using the default filename for the currently selected FLASH.
    fsSaveFile = LittleFS.open(_flashFileName, "w");

    // Print a console message showing the filename with the leading "/" removed from the name.
    console->printf("Saving 64Kbytes of FLASH contents to file %s.\r\n", _flashFileName.substring(1, _flashFileName.length()));
    // Read the Flash a block at a time saving data each block. 
    for (address = 0; address < sizeToRead; address = address + BLOCK_SIZE)
    {
        readOneFlashBlock(address, BLOCK_SIZE, false);
        fsSaveFile.write(_readBuffer, BLOCK_SIZE);
        console->printf(".");
    }
    fsSaveFile.close();
    console->printf("\r\nFinished.\r\n");
}

/**
 * @brief Set up the environemt to start programming the Flash
 * @param fileName the filename which holds the data. If empt then a 
 * default filename is used.
 * 1. Validate the filesystem and file name.
 * 2. If console mode transfer the file to ESP using xmodem.
 * 3. Confirm that we can open the file for reading.
 * 4. Erase the whole Z80 Flash.
 * 5. Set the programming task to active.
 * To do : use better return error codes
*****************************************************************************/
int wtmProgrammer::beginProgramming(char* fileName)
{
    _numBytesToProgram = 0;
    _addr = 0;

    console->printf("\r\n");

    // Enable accessing the filesystem
    if(!LittleFS.begin()){
        console->printf("An Error has occurred while mounting LittleFS\r\n");
        return -1;
    }

    String strFileName = String(fileName);

    // Clean up the filename, open the file, and print name and size to the console
    if (strFileName == "") strFileName = _flashFileName;
    if (strFileName.startsWith("/") == false) 
    {
        strFileName = "/" + strFileName;
    }

    // Locate binary file to program
    // If _programmingFromWeb = True then we already have the file we need stored
    //     on local ESP Flash
    // If _programmingFromWeb = False then we get the file over the serial terminal
    //     and then store that to local ESP Flash
    if (!_programmingFromWeb)
    {
        strFileName = _flashFileName;
        // Here we use xmodem via the Serial port to store the file to ESP local Flash
        // xmodem protocol transfers data in 128 byte segments, so we may get some
        //    extra repeated data on the last packet to pad the packet.
        console->printf("\r\n");
        console->printf("send firmware file %s with xmodem.\r\n", strFileName);
        xmodemRecvFile(strFileName.c_str());
    } else {
        
    }

    // first try to open data file
    _programFile = LittleFS.open(strFileName, "r");

    // Check that there is data in the file image
    int imageSize = _programFile.size();
    if ( !(_programFile) || (imageSize < 1))
    {
        console->printf("There was an error opening file %s.\r\n",  strFileName);
        return 2;
    }

    console->printf("Program file name: %s, size = %d bytes.\r\n", strFileName, _programFile.size());

    if (imageSize > _flashSize)
    {
        imageSize = _flashSize;
        console->printf("The image size = %d bytes, which is greater than the FLASH size. Truncating image.\r\n", strFileName, imageSize);
    }

    selectBus();

    // Read the Flash product ID to make sure it is one
    // supported.
	bool goodFlashId = readFlashId();

    // Erase Flash
    console->printf("\r\n");
    eraseFlash();

    if (_verboseProgramming)
    {
        // Read first 0x100 bytes of Flash to check it is erased
        console->printf("\r\n");
        console->printf("Read back %d bytes of Flash to confirm they are erased (0xFF)\r\n", BLOCK_SIZE);
        readOneFlashBlock(0, BLOCK_SIZE, true);
        console->printf("\r\n");
    }

    console->printf("\r\nWriting Flash with %.2fkB..\r\n", double(imageSize) / 1024);
    _numBytesToProgram = imageSize;
    _addr = 0;
    _programmingActive = true;
    _msStartTime = millis();

    return 0;

}

/**
 * @brief This is the task that programs the flash in blocks.
 * @return true if we were able to program and it was successful.
 * If we reached the end of the flash then close the file and end programming.
*****************************************************************************/
bool wtmProgrammer::updateProgramming()
{
    // Return false if we are not currently programming. That way we
    // can call this continuously to check if there is a programming
    // task in progress.
    if (_programmingActive == false)
    {
        return false;
    }

    // Return false if there was a fault with the file
    if(!_programFile)
    {
        return false;
    }

    bool success = false;
    uint8_t fileBuffer[BLOCK_SIZE];

    size_t byteCount = _programFile.readBytes((char*)fileBuffer, BLOCK_SIZE);

    if ( (byteCount > 0) && (_boolEndFlash == false) )
    {
        // Program the next block of flash
        success = programBlock(fileBuffer, byteCount);
    } else {
       // Programming has finished so finish up.
        console->printf("\r\n\n");
        console->printf("Programming complete.\r\n");
        endProgramming();
        _programmingActive = false;
        _boolEndFlash = false;
        _programFile.close();
        _programFile = File();
        setProgrammingFromWeb(false);

        // Keep the file for later download in case the user wants to 
        // check the file that was used.
        // Todo: put back in once we can read the whole flash.
        // LittleFS.remove(_flashFileName);
    }

    return success;
}

/**
 * @brief Finish programming
*****************************************************************************/
void wtmProgrammer::endProgramming()
{
    // Lets show how long that took
    int msEnd = millis();
    console->printf("\r\n100%% total programming time: %d ms\r\n", msEnd - _msStartTime);

    releaseBus();

    console->printf("\r\n");
}

/**
 * @brief Determines how much of programming is complete. Used during web 
 * programming so that a web message can be staged showing percentage complete.
*****************************************************************************/
int wtmProgrammer::getProgrammingPercentage()
{
    if (_programmingFromWeb == false)
    {
        return 100;
    }

    if (_numBytesToProgram == 0)
    {
        console->printf("\r\nError, programming file size = 0\r\n");
        return 0;
    }
    return 100*_addr/_numBytesToProgram;
    
}

/**
 * @brief Toggle the verbose prints flag. Can be set using the console menu. 
*****************************************************************************/
void wtmProgrammer::toggleVerboseProgramming()
{
    _verboseProgramming = !_verboseProgramming;
}


/**
 * @brief  Detects whether the verbose printing is set. Used for displaying 
 * debug printing mode in the console menu
 * @return true is verbose printing is set.
*****************************************************************************/
bool wtmProgrammer::getVerboseProgramming()
{
    return _verboseProgramming;
}

/**
 * @brief  Debug routine to write just one byte to Z80 Flash and then read
 * the value back from Flash.
 * The z80 bus needs to be initialised before calling this and released
 * after.
*****************************************************************************/
void wtmProgrammer::debugWriteRead()
{
    int      iic = 0;
    // Static variable so next time we use the next address
    // as Flash can only be programmed once without being erased
    static uint16_t addr = 0;

    // Program one byte
    flash_program_byte(iic, addr, 0x55);
    // Read that byte back
    uint8_t byte = bus_read_cycle(iic, addr);
    console->printf("Byte read back at addr 0x%04x = 0x%02x\r\n", addr++, byte);
}


/**
 * @brief Get the current Flash address. Used in the console menu.
 * @return the address.
*****************************************************************************/
uint16_t wtmProgrammer::getReadAddr()
{
    return _readAddr;
}

/**
 * @brief Set the Z80 Flash address. Used in the console menu.
*****************************************************************************/
void wtmProgrammer::zeroReadAddr()
{
    _readAddr = 0;
}

/**
 * @brief Increment the Flash address by one block. Used in the console 
 * menu where it is helpful when reading blocks of data.
*****************************************************************************/
void wtmProgrammer::incReadAddr()
{
    _readAddr += BLOCK_SIZE;
    if (_readAddr > _flashSize) {
        _readAddr = 0;
    }
}
