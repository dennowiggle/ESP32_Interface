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
#ifndef __WTM_PROGRAMMER_H__
#define __WTM_PROGRAMMER_H__

#include <stdint.h>
#include <LittleFS.h>

#define BLOCK_SIZE (0x100)          // block size for accessing the flash = 256 bytes
#define Z80_FLASH_SIZE (0x10000)    // 65,536 bytes supported
#define FPGA_FLASH_SIZE (0x400000)  // 4 Mbytes supported
#define FPGA_IMAGE_SIZE (655690)    // 


class wtmProgrammer 
{
public:
    // Declare a type for holding which flash is selected
    enum wtmFlashType 
    {   Z80_FLASH = 0, 
        FPGA_FLASH = 1 
    };

    // Constructor with initializer
    wtmProgrammer() 
    // The type of Flash that is currently set
    : _flashType(Z80_FLASH)
    // The size of the Flash chip.
    , _flashSize(Z80_FLASH_SIZE)
    // The default filename to be used for programming
    , _flashFileName("/z80retro.bin")
    // Current address for writing data
    , _addr(0)     
    // Address for reading data
    , _readAddr(0) 
    // Number of bytes to program. Normally equal to programming file size.
    , _numBytesToProgram(0) 
    // Start time set before programming data. Helpful for optimising.
    , _msStartTime(0)
    // Set if programming from web interface so telnet/console does not get blocked.
    // If not set Xmodem file download is assumed.
    , _programmingFromWeb(false)
    // Set when programming is active. This is how the main loop knows whether to 
    // continue writing data from file to flash.
    , _programmingActive(false)
    // Set when debugging the programming step. Can be set in console menu.
    , _verboseProgramming(false)
    // Set if we've reached the last byte in the flash.
    , _boolEndFlash(false)
    {};

    ~wtmProgrammer() {};

    // Get and set the type of Flash
    uint8_t getFlashType();
    void setFlashType(wtmFlashType setting);

    // Functions related to Z80 and the Z80 bus
    void resetZ80();
    void selectBus();
    void releaseBus();

    // Read the Flash manufacturer and device ID and return valid status
    bool readFlashId();

    // Functions related to programming and reading the Flash
    int eraseFlash();
    void readOneFlashBlock(uint32_t raddr, uint16_t nbytes, bool bVerbose);
    bool programBlock(uint8_t* block, int numBytes);
    void programFlashFirmware();
    void save64kFlashToFile();

    // Selects the serial or telent port to use for xmodem
    void selectXmPort(uint8_t portId);
    
    // Functions related to programming
    void setProgrammingFromWeb(bool state);
    int beginProgramming(char* filename);
    bool updateProgramming();
    void endProgramming();
    int getProgrammingPercentage();

    // Debug utilities
    void toggleVerboseProgramming();
    bool getVerboseProgramming();
    void debugWriteRead();

    // Provides a way to set the Flash address for reading during console mode
    uint16_t getReadAddr();
    void zeroReadAddr();
    void incReadAddr();

private:

    void selectZ80Bus();
    void releaseZ80Bus();
    bool readZ80FlashId();
    int eraseZ80Flash();
    int verifyFlashBlank(uint64_t startAddr, uint64_t endAddr);
    void writeZ80FlashBlock(uint16_t addr, uint8_t* buffer, uint16_t nbytes);

    wtmFlashType _flashType;
    uint32_t _flashSize;
    String _flashFileName;
    uint8_t _readBuffer[BLOCK_SIZE];
    uint32_t _addr;
    uint32_t _readAddr;
    uint32_t _numBytesToProgram;
    int _msStartTime;
    bool _programmingFromWeb;
    bool _programmingActive;
    bool _verboseProgramming;
    bool _boolEndFlash;
    File _programFile;
};

#endif /* __WTM_PROGRAMMER_H__ */