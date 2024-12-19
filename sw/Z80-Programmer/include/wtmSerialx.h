/*************************************************************************
 * 
 *    Code by Denno Wiggle aka WTM
 * 
 *    Serialx is a WTM class to enable serial port virtualisation so 
 *    that a serial stream can be sent to any uart or telnet port.
 *   
 *    Up to three uart ports and two telnet ports are supported.
 *    It should be possible to increase the number of telnet ports
 *    fairly easily.
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
#ifndef __WTM_SERIAL_X__
#define __WTM_SERIAL_X__

#include <stdint.h>
#include <stdbool.h> 
#include <Arduino.h>
#include "defines.h"


// Used to define which ports are active active in _myPort
// by selecting which bits are enable
#define SX_UART_A       (1 << 0)
#define SX_UART_B       (1 << 1)
#define SX_UART_C       (1 << 2)
#define SX_TELNET_A     (1 << 3)
#define SX_TELNET_B     (1 << 4)
#define SX_TELNET_PORTS (SX_TELNET_A | SX_TELNET_B)


// Code supports up to three uart ports.
// External user code selects how these map to real serial uart ports
// e.g.
//      HardwareSerial *uartA = new HardwareSerial(0);
// where (0) defines Serial 0.
extern HardwareSerial *uartA;
extern HardwareSerial *uartB;
extern HardwareSerial *uartC;


class wtmSerialX {
    
private:
    uint8_t _myPort;
    uint8_t _readBytesPortId;
    uint8_t _telnetBinaryMode;

public:
    bool oneByteOnlyMode;
    bool boolUartASwapped;
    bool fileTransferMode;

typedef enum ebaudRate
{
    baud300 = 300,
    baud600 = 600,
    baud1200 = 1200,
    baud2400 = 2400,
    baud4800 = 4800,
    baud9600 = 9600,
    baud14400 = 14400,
    baud19200 = 19200,
    baud28800 = 28800,
    baud38400 = 38400,
    baud57600 = 57600,
    baud115200 = 115200,
    baudMin = baud300,
    baudMax = baud115200
} baudRate_t;
baudRate_t baudRate;


private:
    size_t sxReadBytes(uint8_t myPort, char* buffer, size_t size);
    int sxAvailable(uint8_t myPort);
    int sxAvailableForWrite(uint8_t myPort);
    int sxWaitUntilNBytesAvailable(uint8_t myPort, size_t size, uint64_t msTimeout);
    int sxParseTelnetControlChars(int clientNum, char* buffer, int dataSize);
    void dumpPacket(char packet[], unsigned int pktLen);
    size_t sendTelnetTxPacket(const uint8_t *buffer, size_t len, int clientNum);
    size_t sxWrite(const uint8_t *buffer, size_t size);

public:
    wtmSerialX();
    ~wtmSerialX();

    uint8_t portCfg();
    uint8_t enablePort(uint8_t enPattern);
    uint8_t disablePort(uint8_t disPattern);
    uint8_t setTelnetBinaryMode(uint8_t setPattern);
    uint8_t unsetTelnetBinaryMode(uint8_t unsetPattern);


/****************************************************************************
 * readBytes() is a helper function coded as two functions 
 * 1. Use the class configured portId configured bit pattern
 * OR
 * 2. Use the a specified portId bit pattern
 * 
 * On top of this both functions are overloaded here to accept char* or 
 * uint8_t* buffer pointers for a total of 4 functions. All of the functions 
 * call common code sxReadBytes() which is the one tht dos the actual work.
****************************************************************************/
    size_t readBytes(char* buffer, size_t size);
    size_t readBytes(uint8_t* buffer, size_t size) ;
    size_t readBytes(char* buffer, size_t size, uint8_t portId);
    size_t readBytes(uint8_t* buffer, size_t size, uint8_t portId);

    uint8_t readPortId();
    int available();
    int availableForWrite();

    void flush();
    size_t printf(const char *format, ...);
    size_t write(const char *buffer, size_t size);
    size_t write(const uint8_t *buffer, size_t size);

};

#endif  /* __WTM_SERIAL_X__ */