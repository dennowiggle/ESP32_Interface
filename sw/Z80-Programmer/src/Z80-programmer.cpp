 /****************************************************************************
 * 
 *    Z80-Retro! ESP Programmer
 *    Code & ESP Programmer Board by Denno Wiggle
 *    Z80-Retro project by John Winans
 *     - https://github.com/Z80-Retro
 * 
 *****************************************************************************
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
#include <HardwareSerial.h>
#include <stdio.h>
#include <LittleFS.h>
#include <EEPROM.h>

#include "defines.h"
#include "wtmSPi.h"
#include "version.h"
#include "wtmProgrammer.h"
#include "wServer.h"
#include "wtmSerialx.h"
#include "telnetServer.h"
#include "Z80IoBus.h"
// Todo : delete when we no longer need to test FPGA flash
#include "wtmFpga.h"

// Todo : DELETE
#include "sdcardtest.h"
#include "flash.h"



#ifdef WTM_PROG_I2C
#include <Wire.h>
#endif

#ifdef ESP32_TYPE_MCU
#include <WebServer.h>
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#endif /* ESP2866_TYPE_MCU */

#ifdef ESP2866_TYPE_MCU
#include <ESP8266WebServer.h>
#endif /* ESP32_TYPE_MCU */

#include <WiFiClient.h>

// UART Port mapping
// Map Console port to Serial1 / uart 1 Tx/Rx
HardwareSerial *uartA = new HardwareSerial(1);
// Map logger to Serial / uart 0 Tx Only
HardwareSerial *uartB = new HardwareSerial(0);
#ifdef ESP32_TYPE_MCU
// Map Z80 to Serial2 / uart 2 Tx/Rx
HardwareSerial *uartC = new HardwareSerial(2);
#else
HardwareSerial *uartC = &Serial;
#endif

wtmProgrammer flashProgrammer;
wtmSerialX* console;
wtmSerialX* logger;
// wtmSerialX* telnetSerial;
wtmSerialX* z80uart;
wtmSerialX* xmodemUart;



typedef enum mode
{
    MENU,
    PROGRAMMING,
    DEBUG,
    CHANGE_BAUD
} Mode;
Mode _mode;


uint8_t ledState = LED_OFF;

void printFileContentsAsText(char *filename)
{
    if(!LittleFS.begin()){
        console->printf("An Error has occurred while mounting LittleFS\r\n");
        return;
    }
  
    File file = LittleFS.open(filename, "r");
    if(!file){
        console->printf("Failed to open file for reading\r\n");
        return;
    }
  
    console->printf("File Content:\r\n");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

// showLittleFsInfo() taken from : 
// https://www.mischianti.org/2020/06/22/wemos-d1-mini-esp8266-integrated-littlefs-filesystem-part-5/
// website has a list of commands available in LittleFS
#ifndef ESP32   // Not supported in ESP32
void showLittleFsInfo()
{
    console->printf("Inizializing FS...");
    if (LittleFS.begin()){
        console->printf("done.\r\n");
    }else{
        Serial.println("fail.\r\n");
    }

    // To format all space in LittleFS
    // LittleFS.format()

    // Get all information of your LittleFS
    FSInfo fs_info;
    LittleFS.info(fs_info);

    console->printf("File sistem info.\r\n");
    console->printf("Total space:      %d bytes\r\n", fs_info.totalBytes);
    console->printf("Total space used: %d bytes\r\n", fs_info.usedBytes);
    console->printf("Block size:       %d bytes\r\n", fs_info.blockSize);
    console->printf("Page size:        %d bytes\r\n", fs_info.totalBytes);
    console->printf("Max open files:   %d\r\n", fs_info.maxOpenFiles);
    console->printf("Max path length:  %d\r\n\n", fs_info.maxPathLength);

    console->printf("File - Size :\r\n");
    // Open dir folder
    Dir dir = LittleFS.openDir("/");
    // Cycle all the content
    while (dir.next()) {
        // get filename
        console->printf("   %s - ", dir.fileName().c_str());
        // If element have a size display It else write 0
        if(dir.fileSize()) {
            File f = dir.openFile("r");
            console->printf("%d\r\n",f.size());
            f.close();
        }else{
            console->printf("0\r\n");
        }
    }
}

void testLittleFS()
{
    console->printf("\r\nTest LittleFS\r\n");
    console->printf("-------------\r\n");
    showLittleFsInfo();
    //printFileContentsAsText((char *)"/text.txt");
}
#endif

#ifdef WTM_PROG_I2C
void scanI2cBus()
{
    uint8_t error, address;
    int nDevices = 0;

    console->printf("\r\nScanning...\r\n");
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            console->printf("I2C device found at address 0x%2x\r\n", address);
            nDevices++;
        }
        else if (error == 4) 
        {
            console->printf("Unknow error at address 0x%2x\r\n", address);
        }
    }
    if (nDevices == 0) {
        console->printf("No I2C devices found\r\n");
    } else {
        console->printf("done\r\n\n");
    }
}
#endif
void displayMenu() {
    // So we know what version we are running
    console->printf("\r\n###########################\r\n");
    console->printf("#  Z80 Retro! Programmer  #\r\n");
    console->printf("###########################\r\n");
    //console->printf("\n----------------\r\n");

    if (_mode == MENU)
    {
        console->printf("Main Menu\r\n");
        console->printf("----------------\r\n");        
        console->printf("p for programming mode\r\n");
        console->printf("d for debug mode\r\n");
#ifndef ESP32_TYPE_MCU
        // ESP2866 share console and z80uart so cannot cross connect the two
        // so need to remove that option
        uint8_t conCfg = console->portCfg();
        console->disablePort(SX_UART_A);
#endif
        console->printf("z to connect Z80 UART\r\n");
        if (z80uart->portCfg() > 0) {
            console->printf("o to disconnect Z80 UART and reconnect console on Serial 0 (USB)\r\n");
        }

#ifndef ESP32_TYPE_MCU
        console->enablePort(conCfg);
#endif
        console->printf("r to reset Z80 Retro\r\n");
        console->printf("b to change Z80 uart baudrate [%d]\r\n", z80uart->baudRate);
        console->printf("c to clear wifi settings\r\n");
        console->printf("w to display wifi settings\r\n");
    }
    else if (_mode == CHANGE_BAUD)
    {
        console->printf("Change Z80 Uart Baud Rate Menu [%d]\r\n", z80uart->baudRate);
        console->printf("---------------------------------------\r\n");        
        console->printf("0 =    300\r\n");
        console->printf("1 =    600\r\n");
        console->printf("2 =   1200\r\n");
        console->printf("3 =   2400\r\n");
        console->printf("4 =   4800\r\n");
        console->printf("5 =   9600\r\n");
        console->printf("6 =  14400\r\n");
        console->printf("7 =  19200\r\n");
        console->printf("8 =  28800\r\n");
        console->printf("9 =  38400\r\n");
        console->printf("a =  57600\r\n");
        console->printf("b = 115200\r\n");
        console->printf("m to return to menu\r\n");
    }
    else if (_mode == PROGRAMMING)
    {
        uint16_t addr = flashProgrammer.getReadAddr();
        uint8_t flashType = flashProgrammer.getFlashType();
        console->printf("Programming Menu (%s)\r\n", 
                (flashType == flashProgrammer.Z80_FLASH) ? "Z80 Flash" : "FPGA Flash");
        console->printf("-----------------------------\r\n"); 
        // xmodem is only supported on UARTA or telnet ports
        uint8_t conCfg = console->portCfg();
        console->disablePort(0xFF & ~(SX_UART_A | SX_TELNET_PORTS) );
        console->printf("c to change Flash to %s\r\n",
                (flashType == flashProgrammer.Z80_FLASH) ? "FPGA Flash" : "Z80 Flash");
        console->printf("p to program (requires a terminal capable of xmodem)\r\n");
        console->enablePort(conCfg);
        console->printf("a to advance address [%04x]\r\n", addr);
        console->printf("z to zero address [%04x]\r\n", addr);
        console->printf("r to read [%04x]\r\n", addr);
        console->printf("e to erase\r\n");
        console->printf("d to debug write/read\r\n");
        console->printf("v to toggle verbose programming (%s)\r\n",
                    flashProgrammer.getVerboseProgramming()? "ON" : "OFF");
        console->printf("m to return to menu\r\n");
    }
    else if (_mode == DEBUG)
    {
        console->printf("Debug Menu\r\n");
        console->printf("----------------\r\n");        
#ifdef WTM_PROG_I2C
        console->printf("s to scan i2c bus\r\n");
#endif
        console->printf("f to read FPGA flash ID\r\n");
        console->printf("i to read Z80 flash ID\r\n");
        console->printf("t to test Fpga flash\r\n");
#ifndef ESP32_TYPE_MCU  // Not supported on ESP32         
        console->printf("f to show LittleFS contents\r\n");
#endif
        console->printf("d to test micro SD card\r\n");
        console->printf("r to read the Z80 message port\r\n");
        console->printf("w to write to the Z80 message port\r\n");
        console->printf("m to return to menu\r\n");
    }
    console->printf("----------------\r\n\n");
}

void menuCommand(unsigned char opt)
{
    uint8_t portId;
    uint8_t portCfg;
    switch (opt)
    {
        case 'p':
            _mode = PROGRAMMING;
            break;
        case 'd':
            _mode = DEBUG;
            break;
        case 'z':
            portId = console->readPortId();

            // Close console on this port and connect Z80 UART to it
            // Print a closing message only to port that commanded 'z'
            portCfg = console->portCfg();
            console->disablePort(portCfg & ~portId);
            console->printf("\r\n***** ESP User terminal disconnected *****\r\n");
            console->enablePort(portCfg);
            // Now turn off the console for the port that commanded 'z'
            console->flush();
            console->disablePort(portId);
#ifdef ESP2866_TYPE_MCU
            swapUartAUartC(true);
#endif
            // enable z80uart cross connect on port where 'z' read
            z80uart->enablePort(portId);
            // Send the connection message to all active users
            z80uart->printf("\r\n***** Z80 User terminal connected    *****\r\n");
            break;

        case 'o':
            if (z80uart->portCfg() & SX_UART_A)
            {
                // Close access to z80uart on port A and connect the ESP console to it
                // Print a closing message on the z80uart channel for uart A only.
                portCfg = z80uart->portCfg();
                z80uart->disablePort(portCfg & ~SX_UART_A);
                z80uart->printf("\r\n***** Z80 User terminal disconnected *****\r\n");
                z80uart->enablePort(portCfg);
                z80uart->flush();
                z80uart->disablePort(SX_UART_A);
#ifdef ESP2866_TYPE_MCU
                swapUartAUartC(false);
#else
                // Reconnect console
                // Enable console on uart A
                portCfg = console->portCfg();
                console->disablePort(portCfg);
                console->enablePort(SX_UART_A);
                console->printf("\r\n***** ESP User terminal connected    *****\r\n");
                console->enablePort(portCfg);
#endif
            }
            break;
        case 'b':
            _mode = CHANGE_BAUD;
            break;
        case 'c':
            clearWifiSettings();
            break;
        case 'r':
            flashProgrammer.resetZ80();
            break;
        case 'w':
            displayWifiSettings();
            break;
        default:
            break;
    }
}


void programCommand(unsigned char opt)
{
    // int rc = 0;
    uint16_t addr = 0;
    uint8_t portId;
    bool oneByteMode;

    switch (opt)
    {
        case 'c':
            if (flashProgrammer.getFlashType() == flashProgrammer.Z80_FLASH)
            {
                flashProgrammer.setFlashType(flashProgrammer.FPGA_FLASH);
            } else {
                flashProgrammer.setFlashType(flashProgrammer.Z80_FLASH);
            }
            break;
        case 'p':
            portId = console->readPortId();
            // Can only program with xmodem on UART A or telnet ports
            //if (portId == SX_UART_A) 
            if ( (portId == SX_UART_A) || (portId & SX_TELNET_PORTS) )
            {
                flashProgrammer.selectXmPort(portId);
                oneByteMode = console->oneByteOnlyMode;
                console->oneByteOnlyMode=false;
                console->flush();
                if (portId & SX_TELNET_PORTS) xmodemUart->fileTransferMode = true;
                flashProgrammer.programFlashFirmware();
                if (portId & SX_TELNET_PORTS) xmodemUart->fileTransferMode = false;
                console->oneByteOnlyMode = oneByteMode;
            }
            break;
        case 'a':
            addr = flashProgrammer.getReadAddr();
            flashProgrammer.incReadAddr();
            break;
        case 'z':
             flashProgrammer.zeroReadAddr();
            break;
        case 'r':
            addr = flashProgrammer.getReadAddr();
            flashProgrammer.selectBus();
            flashProgrammer.readOneFlashBlock(addr, BLOCK_SIZE, true);
            flashProgrammer.releaseBus();
            flashProgrammer.incReadAddr();
            break;        
        case 's':
            flashProgrammer.selectBus();
            flashProgrammer.save64kFlashToFile();
            flashProgrammer.releaseBus();
            break;        
        case 'e':
            flashProgrammer.selectBus();
            flashProgrammer.eraseFlash();
            flashProgrammer.releaseBus();
            break;
        case 'd':
            flashProgrammer.selectBus();
            flashProgrammer.debugWriteRead();
            flashProgrammer.releaseBus();
            break;
        case 'b':
            break;
        case 'v':
            flashProgrammer.toggleVerboseProgramming();
            break;
        case 'm':
            _mode = MENU;
            break;
        default:
            break;
    } 
}


void debugCommand(unsigned char opt)
{
    switch (opt)
    {
        case 'f':
            flashProgrammer.setFlashType(flashProgrammer.FPGA_FLASH);
            flashProgrammer.selectBus();
            flashProgrammer.readFlashId();
            flashProgrammer.releaseBus();
            break;

        case 'i':
            flashProgrammer.setFlashType(flashProgrammer.Z80_FLASH);
            flashProgrammer.selectBus();
            flashProgrammer.readFlashId();
            flashProgrammer.releaseBus();
            break;

        case 't':
            flashProgrammer.setFlashType(flashProgrammer.FPGA_FLASH);
            flashProgrammer.selectBus();
            testFpgaFlash();
            flashProgrammer.releaseBus();
            break;

#ifdef WTM_PROG_I2C
        case 's':
            // scan i2c bus
            scanI2cBus();
            break;
#endif
#ifndef ESP32_TYPE_MCU    // Not supported on ESP32          
        case 'f':
            // show LittleFS contents
            testLittleFS();
            break;
#endif
        case 'd':
            console->printf("\r\n###################################\r\n");
            console->printf("Test the SPI bus without SD Card SW\r\n");
            console->printf("###################################\r\n");
            lowLevelSpiTest(false);
            delay(1000);
            console->printf("\r\n###################################\r\n");
            console->printf("Test the SPI bus with SD Card SW\r\n");
            console->printf("###################################\r\n");
            lowLevelSpiTest(true);
            break;
        case 'r':
            console->printf("\r\nZ80 Data 8-bit data port reads 0x%02x\r\n", z80DataPortRead());
            break;
        case 'w':
        {
            uint8_t z80data = 0x55;
            console->printf("\r\nZ80 Data 8-bit data port write 0x%02x\r\n", z80data);
            z80DataPortwrite(z80data);
        }
            break;
        case 'm':
            _mode = MENU;
            break;
        default:
            break;
    }

}

void changeBaudCommand(unsigned char opt)
{
    wtmSerialX::baudRate_t oldBaud = z80uart->baudRate;
    wtmSerialX::baudRate_t newBaud = oldBaud;
    switch (opt)
    {
        case '0':
            newBaud = z80uart->baud300;
            break;
        case '1':
            newBaud = z80uart->baud600;
            break;
        case '2':
            newBaud = z80uart->baud1200;
            break;
        case '3':
            newBaud = z80uart->baud2400;
            break;
        case '4':
            newBaud = z80uart->baud4800;
            break;
        case '5':
            newBaud = z80uart->baud9600;
            break;
        case '6':
            newBaud = z80uart->baud14400;
            break;
        case '7':
            newBaud = z80uart->baud19200;
            break;
        case '8':
            newBaud = z80uart->baud28800;
            break;
        case '9':
            newBaud = z80uart->baud38400;
            break;
        case 'a':
            newBaud = z80uart->baud57600;
            break;
        case 'b':
            newBaud = z80uart->baud115200;
            break;
        case 'm':
            _mode = MENU;
            break;
        default:
            break;
    }

    if (newBaud != oldBaud)
    {
        z80uart->baudRate = newBaud;
        EEPROM.put(sizeof(wifiSettings), z80uart->baudRate);
        EEPROM.commit();

        if (z80uart->portCfg() > 0)
        {
            uartC->flush();
            uartC->end();
            uartC->begin(newBaud);
#ifndef ESP32_TYPE_MCU
            if (z80uart->boolUartASwapped)
            {
                uartA->swap();
            }
            // Flush the Rx buffer
            while(uartC->available())
            {
                uartC->read();
            }
#endif
        }
    }

}


void doCommand(unsigned char opt)
{
    if (_mode == MENU)
    {
        menuCommand(opt);
    }
    else if (_mode == PROGRAMMING)
    {
        programCommand(opt);
    }
    else if (_mode == DEBUG)
    {
        debugCommand(opt);
    }
    else if (_mode == CHANGE_BAUD)
    {
        changeBaudCommand(opt);
    }
}


void handleXconnect()
{
    // Check if there is data available from the Z80 retro computer
    // If yes, send it to all the active z80 user terminals
    int afr = (uartC->available());
    int maxToSend = z80uart->availableForWrite();
    bool boolSend = (afr && maxToSend);
    while (boolSend)
    {
        int len = afr;
        len = std::min(len, maxToSend);
        // A max size of 127 byte packets max seems to be optimal for telnet based on testing.
        //len = std::min(len, STACK_PROTECTOR);
        len = std::min(len, 127);
        if (len) 
        {
            uint8_t sbuf[len];
            len = uartC->readBytes(sbuf, len);
            if (len > 0)
            {
                // Debug code to print hex values for the bytes received from the z80 uart
                // logger->printf("Z80 Rx : \r\n");
                // for (int i = 0; i < len; i++)
                // {
                //     logger->printf("%02x ", sbuf[i]);
                // }
                // logger->printf("\r\n");

                z80uart->write(sbuf, len);
            }
        } else {
            logger->printf("Z80 uart congestion: uart = %d bytes, afw = %d bytes \r\n", uartC->available(), z80uart->availableForWrite());
        }

        afr = (uartC->available());
        maxToSend = z80uart->availableForWrite();
        boolSend = (afr && maxToSend);
    }

    // Check if there is data available on any of the user terminals
    // If yes, send it the Z80 retro computer
    // Todo : re-evaluate if vs while performance. When first tested
    // while blocked other tasks from happening.
    if (z80uart->available())
    // while (z80uart->available())
    {
        int len = std::min(z80uart->available(), STACK_PROTECTOR);
        if (len) 
        {
            // Lookup which port had bytes available to read
            uint8_t readPortId = z80uart->readPortId();
            uint8_t sbuf[len];
            // Read the data from the specific port
            len = z80uart->readBytes(sbuf, len, readPortId);

            // Debug code to print hex values for the bytes received from the z80 uart
            // logger->printf("Z80 Tx : \r\n");

            for (int i = 0; i < len; i++)
            {
                // logger->printf("%02x ", sbuf[i]);
                uartC->write(sbuf[i]);
            }
            // logger->printf("\r\n");
        }
    }
}

#if (WTM_REDIRECT_STD_OUT == 1)
// function used to redirect stdout to where we want it
// https://wokwi.com/projects/346783796954137171
ssize_t write_fn(void* cookie, const char* buf, ssize_t size)
{
  /* redirect the bytes somewhere; writing to Serial just for an example */
  console->write((uint8_t*) buf, size);
  return size;
}

// function used to redirect other outputs
// https://wokwi.com/projects/346783796954137171
void ets_putc_handler(char c)
{
  /* this gets called from various ets_printf / esp_rom_printf calls */
  static char buf[256];
  static size_t buf_pos = 0;
  buf[buf_pos] = c;
  buf_pos++;
  if (c == '\n' || buf_pos == sizeof(buf)) {
    /* flush */
    write_fn(NULL, buf, buf_pos);
    buf_pos = 0;
  }
}
#endif

// Called once only at startup
void setup() {

    // Setup the LED
#ifdef USE_ON_BOARD_LED
    #ifdef WTM_RGB_PIN
    pinMode(WTM_RGB_PIN, OUTPUT);
    neopixelWrite(WTM_RGB_PIN,WTM_RGB_BRIGHTNESS,0,0); // Red
    #else /* Not WTM_RGB_PIN */
        pinMode(LED_PIN, OUTPUT);
        ledState = LED_OFF;
        digitalWrite(LED_PIN, ledState);
    #endif
#endif

    // First set up the logging port in cased it needs to be used first
    logger = new wtmSerialX;
    logger->baudRate = LOGGER_BAUD;
    uartB->setRxBufferSize(LOGGER_RX_BUF_SIZE);
    #ifdef ESP32_TYPE_MCU
    uartB->begin(logger->baudRate, SERIAL_8N1, RXD0, TXD0);
    delay(500);
    uartB->printf("Hello uartB\r\n");
    #else
    uartB->begin(logger->baudRate);
    #endif
    logger->enablePort(SX_UART_B);

    // This will be the console port
    console = new wtmSerialX;
    console->baudRate = CONSOLE_BAUD;
    uartA->setRxBufferSize(CONSOLE_RX_BUF_SIZE);
    uartA->begin(console->baudRate, SERIAL_8N1, RXD1, TXD1);
    console->enablePort(SX_UART_A);

    // telnetSerial = new wtmSerialX;
    // Do not set port config until new telnet connection
    // We map the telnet screen when we get a connection
    // telnetSerial->enablePort(0);

    // xmodemUart is used to serially transfer a z80 program for programming
    // This allows us to be fleible where the source comes from - Serial or Telnet
    xmodemUart = new wtmSerialX;

    // EEPROM code to read/write z80 baud rate to EEPROM
    // EEPROM used so that we remember the setting if it gets changed between
    // power cycles.
    // Baud Rate value stored right after Wifi settings
    EEPROM.begin(sizeof(WiFiSettings) + sizeof(z80uart->baudRate));
    int eeAddrZ80BaudRate = sizeof(WiFiSettings);

    z80uart = new wtmSerialX;
    EEPROM.get(eeAddrZ80BaudRate, z80uart->baudRate);
    // Check to see if it is in a valid range. 
    // Should ideally check for a match of each possible value
    // but this should suffice
    if (z80uart->baudRate < wtmSerialX::baudMin ||  z80uart->baudRate > wtmSerialX::baudMax)
    {
        z80uart->baudRate = Z80_BAUD;
        EEPROM.put(eeAddrZ80BaudRate, z80uart->baudRate);
        EEPROM.commit();
    }
#ifdef ESP32_TYPE_MCU
    uartC->setRxBufferSize(Z80_RX_BUF_SIZE);
    uartC->begin(z80uart->baudRate, SERIAL_8N1, RXD2, TXD2);
#endif
    // Leave portCfg = 0 (initialised state) until we are ready to use it)

    // Having been able to get this to work lower down. Seems after wifi is enabled.
    // This is because Tx1 is also the LED pin.
    logger->printf("\r\nUsing Serial1 for logging\r\n");
#ifndef ESP32_TYPE_MCU
    logger->printf("ESP version : %s\r\n",ESP.getFullVersion().c_str());
#else
    logger->printf("ESP version : %s\r\n",ESP.getSdkVersion());
#endif
    logger->printf("Serial baud: %d (8n1: %d KB/s)\r\n", (int)CONSOLE_BAUD, (int)CONSOLE_BAUD * 8 / 10 / 1024);
    logger->printf("Serial receive buffer size: %d bytes\r\n", CONSOLE_RX_BUF_SIZE);
    
    // Need a delay as something below is locking the serail1 port.
    delay(100);

#ifdef WTM_PROG_I2C
    // Set up the i2c bus for talking with the 2x MCP23017
    Wire.begin(SDA, SCL);
    delay(1000);
    // Supported speeds on MCP23017 are 100kHz, 400kHz, 1.7Mhz
    // The ESP2866 only supports 100kHz, 400kHz at driver level
    // TODO: Measure signal integrity of i2c 
    //       and timing of Flash signals
    // Wire.setClock(100000);
    Wire.setClock(400000);
#endif

    // Setup the pins and ports
    z80DataPortDirSetup(INPUT);
    z80IorqSetup();

#ifdef WTM_PROG_SPI
    initSpiMain();
#endif

    // The wifi server uses files stored on local Flash
    LittleFS.begin();

    // A delay here allows time for the user to connect of a UART terminal
    // after programming and still see full output
    delay(5000);
    
    console->printf("\r\n\n\n");
    console->printf("#######################################################\r\n");
    console->printf("#######################################################\r\n\n");
    // console->printf("console port      = 0x%02x\r\n", console->portCfg());
    // console->printf("logger port       = 0x%02x\r\n", logger->portCfg());
    // console->printf("telnetSerial port = 0x%02x\r\n\n", telnetSerial->portCfg());
 
    // So we know what version we are running
    console->printf("Sketch:       = %s\r\n", __FILE__);
    console->printf("Version       = %s\r\n",VERSION);
    // console->printf("Uploaded: %s %s\r\n\n", __DATE__, __TIME__);   
    console->printf("Free memory   = %d kBytes\r\n", ESP.getFreeHeap()/1024);
    console->printf("CPU frequency = %d MHz\r\n", ESP.getCpuFreqMHz());
    console->printf("Flash size    = %d kBytes\r\n", ESP.getFlashChipSize()/1024);
    console->printf("SDK Version   = %s\r\n", ESP.getSdkVersion());
    console->printf("Sketch size   = %d kBytes\r\n", ESP.getSketchSize()/1024);
#ifdef ESP32_TYPE_MCU
    console->printf("Chip model    = %s Rev %d\r\n", ESP.getChipModel(), ESP.getChipRevision());
#else // ESP2866_TYPE_MCU
    console->printf("Chip ID       = 0x%04x\r\n", ESP.getChipId());
    console->printf("Version       = %s\r\n", ESP.getFullVersion().c_str());
#endif // ESP32_TYPE_MCU

#ifdef ESP32_TYPE_MCU
	uint32_t lowMac  = ESP.getEfuseMac() & 0xFFFFFFFF; 
    uint32_t highMac = ( ESP.getEfuseMac() >> 32 ) % 0xFFFFFFFF;
    uint64_t chipId  = ESP.getEfuseMac();

    //console->printf("High + Low Address %04x%08x\r\n", highMac, lowMac);
    console->printf("ChipId        = %012llx\r\n", chipId);
    console->printf("Mac Address   = %02x%02x%02x%02x%02x%02x\r\n", 
                    lowMac & 0xff, 
                    (lowMac >> 8) & 0xff,
                    (lowMac >> 16) & 0xff,
                    (lowMac >> 24) & 0xff,
                    highMac & 0xff,
                    (highMac >> 8) & 0xff);
	console->printf("Core count    = %d\r\n", ESP.getChipCores());
#endif

#ifdef ESP32_S3_DEVKITC
/* To use GPIO45 with no issues We want settings for VDD SPI for 3.3V use
 * 
 * See Table 1-1 in esp32-s3_datasheet_en.pdf
 * Ordering Code    In-Package Flash   In-Package PSRAM    Ambient Temp(°C)  VDD_SPI Voltage
 * ESP32-S3         —                  —                   –40 ~ 105         3.3 V/1.8 V
 * ESP32-S3FN8      8 MB (Quad SPI)    —                   –40 ~ 85          3.3 V
 * ESP32-S3R2       —                  2 MB (Quad SPI)     –40 ~ 85          3.3 V
 * ESP32-S3R8       —                  8 MB (Octal SPI)    –40 ~ 65          3.3 V
 * ESP32-S3R8V      —                  8 MB (Octal SPI)    –40 ~ 65          1.8 V
 * 
 * See Table 7 in esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf
 * EFUSE_VDD_SPI_FORCE    GPIO45    EFUSE_VDD_SPI_TIEH   Voltage VDD_SPI power source
 * 0                      0         Ignored              3.3 V VDD3P3_RTC via RSP I
 * 0                      1         Ignored              1.8 V Flash Voltage Regulator
 * 1                      Ignored   0                    1.8 V Flash Voltage Regulator
 * 1                      Ignored   1                    3.3 V VDD3P3_RTC via RSP I
*/
    /* Test that the fuse bits are set correctly for GPIO45 use */
    uint8_t testBit2 = esp_efuse_read_field_bit(ESP_EFUSE_VDD_SPI_TIEH);
    uint8_t testBit1 = esp_efuse_read_field_bit(ESP_EFUSE_VDD_SPI_FORCE);
    console->printf("ESP_EFUSE_VDD_SPI_FORCE = 0x%02x\r\n", testBit1);
    console->printf("ESP_EFUSE_VDD_SPI_TIEH  = 0x%02x\r\n", testBit2);
    if (testBit1 == 0 && testBit2 == 1) 
    {
        console->printf("*** WARNING - THIS MODULE MAY NOT BE COMATBILE FOR GPIO45 USE \r\n");
        console->printf("            - unless pin is tri-state during reset \r\n");
    }
#endif

    console->printf("\r\n");


#if (WTM_REDIRECT_STD_OUT == 1)
    // Two functions used to redirect stdout to where we want it
    // https://wokwi.com/projects/346783796954137171
    stdout = funopen(NULL, NULL, &write_fn, NULL, NULL);
    static char linebuf[256];
    setvbuf(stdout, linebuf, _IOLBF, sizeof(linebuf));

    /* redirect ets_printf / esp_rom_printf output */
    ets_install_putc1(&ets_putc_handler);
#endif

    // Setup the Wifi server
    startServer();
    startTelnetServer();

    // Menu for direct connect console mode.
    displayMenu();
}

// extern int os_printf_plus(const char * format, ...);
// #define printf(...) os_printf( __VA_ARGS__ )

// Loops forever
void loop() {

    // Handles the setup of Wifi Network in AP mode
    handleClient();

    // Handles the telnet connection.
    handleTelnet();

#ifdef WTM_PROG_SPI
    // Handles the plugging and unplgging of SD cards
    handleSdCardEvents();
#endif

#ifdef WTM_TEST_Z80_MESSAGE_INTERFACE
    // for board bring-up test to test automatic reading and writing to the Z80.
    // Need to run esp32rw.com on the z80 for this code to run. 
    // Low-level board test only! Disables normal IRQ finction.
    handleZ80IorqTest();
#endif

#ifndef WTM_TEST_Z80_MESSAGE_INTERFACE
     // Lets us know the Z80 activity on the IORQ A0 Read and Write pins
   handleZ80Iorq();
#endif

    // Cross connect uartC to another uart or telnet
    if (z80uart->portCfg() > 0)
        handleXconnect();

    // Here we check if there has been a web request to program firmware
    // that we need to go perform
    // If not check the serial port for a command.
    if (!flashProgrammer.updateProgramming())
    {
        // Here we control the console menus.
        if (console->available() > 0)
        {
            logger->printf("console->available() = 0x%02x\r\n", console->available() );
            logger->printf("console->readPortId() = 0x%02x\r\n", console->readPortId() );
            unsigned char b;
            size_t len = console->readBytes(&b, 1, console->readPortId());

            if (len > 0) {
                // Serial.printf("Bytes available = %d\r\n", len);
                bool oneByteMode = console->oneByteOnlyMode;
                console->oneByteOnlyMode=true;
                doCommand(b);
                console->oneByteOnlyMode = oneByteMode;
                displayMenu();
            }
        }
    }
}



