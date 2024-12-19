/*************************************************************************
 * 
 *    Code by Denno Wiggle aka WTM
 * 
 *    wtmSerialx is a class to enable serial port virtualisation so 
 *    that a serial stream can be sent to any uart or telnet client.
 *   
 *    Up to three uart ports and two telnet clients are supported.
 *    It should be possible to increase the number of telnet ports
 *    fairly easily.
 * 
 *    As such this provides wrappers for printf(), readBytes(), write()
 *    etc, so wtmSerialx data can be rediected to one or more ports and 
 *    is agnostic to the type of port either uart or telnet or both.
 * 
 *    There is a requirement for an externally declared telnet Server 
 *    and this code needs to reference externally declared clients.
 * 
 * **************************************************************************
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
#include <algorithm>
#include "defines.h"

#ifdef ESP32_TYPE_MCU
#include <WiFiClient.h>
#endif /* ESP32_TYPE_MCU */

#ifdef ESP2866_TYPE_MCU
#include <WiFiClient.h>
#endif /* ESP2866_TYPE_MCU */

#include "wtmSerialx.h"
#include "dbg.h"

// Needed for external function logTelnetCommandBytes()
// Is a placeholder for possible future improvement.
#include "telnetServer.h"

// This specifies a timeout for Reads
#define SX_READ_TIMEOUT 1000

// Define where debug logging output should be directed
#ifdef SX_TEL_RX_DEBUG
extern wtmSerialX* logger;
// Todo : Document use in other programs e.g.
// extern HardwareSerial* logger;
#endif

extern WiFiClient serverClients[MAX_TELNET_CLIENTS];


/****************************************************************************
 * Constructor for wtmSerialx class with initializer
 ****************************************************************************/
wtmSerialX::wtmSerialX() :
    _myPort               (0)
    , _readBytesPortId    (0)
    , _telnetBinaryMode   (0)
    , oneByteOnlyMode     (true)
    , boolUartASwapped    (false)
    , fileTransferMode    (false)
    , baudRate            (baud115200)
{}


/****************************************************************************
 * Provides the current port config bit pattern
 * @return an 8 bit bit pattern for which uart and telnet ports are active
 ****************************************************************************/
uint8_t wtmSerialX::portCfg()
{
    return _myPort;
}

/****************************************************************************
 * Enable additional bits for the port config bit pattern
 * @param enPattern is the bit pattern for uart/telnet ports to enable
 * @return the bit pattern for which uart and telnet ports are now active
 ****************************************************************************/
uint8_t wtmSerialX::enablePort(uint8_t enPattern)
{
    _myPort = (_myPort | enPattern);
    return _myPort;
}

/****************************************************************************
 * Disable bits in the port config bit pattern
 * @param disPattern is the bit pattern for uart/telnet ports to remove
 * @return the bit pattern for which uart and telnet ports are now active
 ****************************************************************************/
uint8_t wtmSerialX::disablePort(uint8_t disPattern)
{
    _myPort = (_myPort & ~disPattern);
    return _myPort;
}

/****************************************************************************
 * Set additional bit(s) for binary mode on the specifc telnet port(s)
 * @param enPattern is the bit pattern for telnet port(s)
 * @return the bit pattern for which telnet ports are in binary mode
 ****************************************************************************/
uint8_t wtmSerialX::setTelnetBinaryMode(uint8_t enPattern)
{
    _telnetBinaryMode = (_telnetBinaryMode | enPattern);
    return _telnetBinaryMode;
}

/****************************************************************************
 * Unset bit(s) for binary mode on the specifc telnet port(s)
 * @param disPattern is the bit pattern for telnet port(s)
 * @return the bit pattern for which telnet ports are in binary mode
 ****************************************************************************/
uint8_t wtmSerialX::unsetTelnetBinaryMode(uint8_t disPattern)
{
    _telnetBinaryMode = (_telnetBinaryMode & ~disPattern);
    return _telnetBinaryMode;
}


/****************************************************************************
 * readBytes() is a helper function coded as two functions 
 * 1. Use the class configured portId configured bit pattern
 * OR
 * 2. Use the a specified portId bit pattern
 * 
 * On top of this both functions are overloaded here to accept char* or 
 * uint8_t* buffer pointers. All of the functions call common code 
 * sxReadBytes() which is the one that does the actual work.
****************************************************************************/

/****************************************************************************
 * Read a specific number of bytes. Here we don't specify which port to read
 * from, just take the first one that has data available. readPortId() when 
 * called will return the port bit in the bit pattern that we read from.
 * @param buffer the buffer to put the read data
 * @param size the number of bytes to read
 * @return the number of bytes read
 ****************************************************************************/
size_t wtmSerialX::readBytes(uint8_t* buffer, size_t size) 
{
        return sxReadBytes(_myPort, (char*)buffer, size);
}

/****************************************************************************
 * Read a specific number of bytes. readPortId() when called will return the 
 * port bit in the bit pattern that we read from.
 * @param buffer the buffer to put the read data
 * @param size the number of bytes to read
 * @param portId the bit pattern for port or ports to read from
 * @return the number of bytes read
 ****************************************************************************/
size_t wtmSerialX::readBytes(uint8_t* buffer, size_t size, uint8_t portId) 
{
        return sxReadBytes(portId, (char*)buffer, size);
}

/****************************************************************************
 * When bytes are read from a port _readBytesPortId gets updated. This is a 
 * helper function to return the ID of the port which was read from. This is 
 * useful when trying to read from any port and accept data from the first 
 * one with bytes available. The user can then call this function if they 
 * need to know which specific port was read from.
 * @return the port ID bit that was read from
 ****************************************************************************/
uint8_t wtmSerialX::readPortId()
{
    return _readBytesPortId;
}

/****************************************************************************
 * Check which port of the enable ports has bytes avaialable to read 
 * @return the first port ID bit that has bytes avaialable to read
 ****************************************************************************/
int wtmSerialX::available()
{
    return sxAvailable(_myPort);
}

/****************************************************************************
 * Check availability for writing among all active ports configured.   
 * @return the number of bytes available for writing in the tx buffer on the 
 * first port with bytes available in tx fifo.
 ****************************************************************************/
int wtmSerialX::availableForWrite()
{
    return sxAvailableForWrite(_myPort);
}

/****************************************************************************
 * Flush / send the data in the tx fifo buffers of all configured ports 
 * on this channel. This should be done before closing or reconfiguring 
 * a channel.
 ****************************************************************************/
void wtmSerialX::flush()
{
    if (_myPort & SX_UART_A)
    {
        uartA->flush();
    }
    
    if (_myPort & SX_UART_B) 
    {
        uartB->flush();
    }
    
    if (_myPort & SX_UART_C) 
    {
        uartB->flush();
    }
    
    if  (_myPort & SX_TELNET_PORTS) 
    {
            for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
            {
                if ( (_myPort & (SX_TELNET_A << i)) && (serverClients[i].connected()) )
                {
                    serverClients[i].flush();
                }
            }
    }
}

/****************************************************************************
 * This printf() function is a copy of code in HardwareSerial.cpp. The one
 * difference is the change to use the write() function from this wtmSerialx
 * class to output the data to the configured serial and telnet ports.
 * @param printf_format 
 * @return the number of bytes written
 *****************************************************************************/
size_t wtmSerialX::printf(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1) {
        buffer = new (std::nothrow) char[len + 1];
        if (!buffer) {
            return 0;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }

    // Write is the class function to output data to the configured ports.
    len = write(buffer, len);
    if (buffer != temp) {
        delete[] buffer;
    }

    return len;
}


#if defined(SX_TEL_RX_DEBUG) || defined(SX_TEL_TX_DEBUG)

/****************************************************************************
 * Dump a data packet to the debug port for debug purposes.
 * @param packet holds the packet bytes to print.
 * @param pktLen is the number of bytes in the buffer.
 *****************************************************************************/
void wtmSerialX::dumpPacket(char packet[], unsigned int pktLen)
{
    for(unsigned int byteNum = 0; byteNum < pktLen; byteNum++)
    {
        if (byteNum%16 == 0)
        {
            logger->printf("%s%04x:", byteNum==0?"":"\r\n", byteNum);
        }
        logger->printf(" %02x", packet[byteNum]);
    }
    logger->printf("\r\n");
}
#endif

/****************************************************************************
 * Function to send the Telnet Tx packet buffer and add extra escape 
 * characters if in binary mode
 * @param buffer holds the bytes to in the Tx packet.
 * @param len is the number of bytes in the buffer.
 * @return the number of extra bytes to write
 *****************************************************************************/
size_t wtmSerialX::sendTelnetTxPacket(const uint8_t *buffer, size_t len, int clientNum)
{
    size_t sizeWrite = 0;

    // If we are in binary mode we need to escape control characters 255 and 13 + 0
    if (_telnetBinaryMode & _myPort) 
    {
        size_t newLength = 0;
        static uint8_t prevByte = 0;

        // Abort if buffer is too long. 
        if (len > TELNET_MAX_TX_BUF_SIZE)
        {
            console->printf("%s : ERROR : Telnet Tx buffer size of %d bytes is larger than max value %d. Aborting", __FUNCTION__, len, TELNET_MAX_TX_BUF_SIZE);
            logger->printf("%s : ERROR :Telnet Tx buffer size of %d bytes is larger than max value %d. Aborting", __FUNCTION__, len, TELNET_MAX_TX_BUF_SIZE);
            return 0;
        }

        // Declare a new Tx Buffer twice the size of the old one
        uint8_t* tempTxBuffer = new uint8_t[len * 2];

        // Loop through the data one byte at a time. 
        //   - Send an extra 255 for every 255 in the data. 
        //   - Send an extra 0 if 13 followed by 0 has been detected.
        //   - this may take us past the number of bytes available to write
        //   - but have not seen that be an issue to date.
        for (int index = 0; index < len; index++)
        {
            tempTxBuffer[newLength] = buffer[index];
            newLength++;

            if (buffer[index] == 255) 
            {
                tempTxBuffer[newLength] = 255;
                newLength++;
            // Prepare to send an extra 0 byte if 13 followed by 0 has been sent.
            } else if (buffer[index] == 0 & prevByte == 13) {
                tempTxBuffer[newLength] = 0;            
                newLength++;
            }                    
            prevByte = buffer[index];
        }

        sizeWrite = serverClients[clientNum].write(tempTxBuffer, newLength);
        if (sizeWrite != newLength) {
            DBG("len mismatch port 0x%02x: tried to send:%zd actually sent:%zd\r\n", _myPort, len, sizeWrite);
        }
        // Adjust the size written back down to what the user expects
        // sizeWrite = sizeWrite - (newLength - len);

        // Delete the temporary buffer
        delete[] tempTxBuffer;

    } else {
        // Not in binary mode so we can send normally.
        sizeWrite = serverClients[clientNum].write(buffer, len);
        if (sizeWrite != len) {
            DBG("len mismatch port 0x%02x: tried to send:%zd actually sent:%zd\r\n", _myPort, len, sizeWrite);
        }
    }

    // Todo : Remove previous code below once enough testing has taken place

    // Loop through the data sending one byte at a time. 
    // If in Binary mode send an extra 255 for every 255 in the data sent and 
    // also add an extra 0 if 13 and 0 has been sent.
    // Todo : Consider resizing and editing send buffer so packet can be sent all at once.
    // static uint8_t lastByte = 0;
    // for (int index = 0; index < len; index++)
    // {
    //     sizeWrite = sizeWrite + serverClients[clientNum].write(&buffer[index], 1);
    //     // Send an extra 255 byte for every 255 in the data sent. 
    //     if (buffer[index] == 255) 
    //     {
    //         if (_telnetBinaryMode & _myPort) serverClients[clientNum].write(&buffer[index], 1);
    //     // Send an extra 0 byte if 13 and 0 has been sent.
    //     } else if (buffer[index] == 0 & lastByte == 13) {
    //         if (_telnetBinaryMode & _myPort) serverClients[clientNum].write(&buffer[index], 1);
    //     }                    
    //     lastByte = buffer[index];
    // }

    // if (sizeWrite != len) {
    //     DBG("len mismatch port 0x%02x: available to send:%zd actually sent:%zd\r\n", _myPort, len, sizeWrite);
    // }

    return sizeWrite;
}

/****************************************************************************
 * Overload Write function 1 which accepts a buffer of type const char *.
 * Output bytes to the configured serial and telnet ports for this channel. 
 * @param buffer holds the bytes to write.
 * @param len is the number of bytes in the buffer to write.
 * @return the number of bytes written
 *****************************************************************************/
size_t wtmSerialX::write(const char *buffer, size_t len)
{
    size_t sizeWrite = sxWrite((uint8_t *)buffer, len);
    return sizeWrite;
}


/****************************************************************************
 * Overload Write function 2 which accepts a buffer of type const uint8_t.
 * Output bytes to the configured serial and telnet ports for this channel. 
 * @param buffer holds the bytes to write.
 * @param len is the number of bytes in the buffer to write.
 * @return the number of bytes written
 *****************************************************************************/
size_t wtmSerialX::write(const uint8_t *buffer, size_t len)
{
    size_t sizeWrite = sxWrite(buffer, len);
    return sizeWrite;
}

/****************************************************************************
 * Output bytes to the configured serial and telnet ports for this channel. 
 * @param buffer holds the bytes to write.
 * @param len is the number of bytes in the buffer to write.
 * @return the number of bytes written
 *****************************************************************************/
size_t wtmSerialX::sxWrite(const uint8_t *buffer, size_t len)
{
    size_t sizeWrite = 0;

    if (_myPort & SX_UART_A)
    {
        sizeWrite = uartA->write((const uint8_t*) buffer, len);
        if (sizeWrite != len) {
            DBG("len mismatch port 0x%02x: available to send:%zd actually sent:%zd\r\n", SX_UART_A, len, sizeWrite);
        }
    } 
    
    if (_myPort & SX_UART_B) 
    {
        sizeWrite = uartB->write((const uint8_t*) buffer, len);
        if (sizeWrite != len) {
            DBG("len mismatch port 0x%02x: available to send:%zd actually sent:%zd\r\n", SX_UART_B, len, sizeWrite);
        }
    }
    
    if (_myPort & SX_UART_C) 
    {
        sizeWrite = uartC->write((const uint8_t*) buffer, len);
        if (sizeWrite != len) {
            DBG("len mismatch port 0x%02x: available to send:%zd actually sent:%zd\r\n", SX_UART_C, len, sizeWrite);
        }
    } 
    
    if (_myPort & SX_TELNET_PORTS) 
    {
#ifdef SX_TEL_TX_DEBUG
                logger->printf("Dump Tx packet\r\n");
                // logger->printf("Dump Tx packet (_telnetBinaryMode = 0x%02x)\r\n", _telnetBinaryMode);
                dumpPacket((char *) buffer, len);
#endif
        for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
        {
            // availableForWrite() returns 0 with ESP32
            // To avoid this use SerialX.avaialableForWrite() function before calling this function.
            // and here we use connected().
            if ( (_myPort & (SX_TELNET_A << i)) && (serverClients[i].connected()) )
            {
                sizeWrite = sendTelnetTxPacket((const uint8_t*) buffer, len, i);
            }
        }
    }
    return sizeWrite;
    
}

/****************************************************************************
 * Check which port has bytes avaialable to read 
 * @param myPort the bit pattern for port or ports to check
 * @return the first port ID bit that has bytes avaialable to read
 ****************************************************************************/
int wtmSerialX::sxAvailable(uint8_t myPort)
{
    int available = 0;

    if (myPort & SX_UART_A)
    {
        available = uartA->available();
        if (available > 0)
        {
            _readBytesPortId = SX_UART_A;
            return available;
        }
    }
    
    if (myPort & SX_UART_B) 
    {
        available = uartB->available();
        if (available > 0)
        {
            _readBytesPortId = SX_UART_B;
            return available;
        }
    }
    
    if (myPort & SX_UART_C) 
    {
        available = uartC->available();
        if (available > 0)
        {
            _readBytesPortId = SX_UART_C;
            return available;
        }
    }
    
    if (myPort & SX_TELNET_PORTS) 
    {
        for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
        {
            if (  (myPort & (SX_TELNET_A << i)) && serverClients[i].available() ) {
                available = serverClients[i].available();
                if (available > 0)
                {
                    _readBytesPortId = (SX_TELNET_A << i);
                    return available;
                }
            }
        }
    }
    // If we get here it means there is no available data to be read
    return available;
}

/****************************************************************************
 * Check if the port specified is avaialable for writing. If there are 
 * multiple ports specified in the bit pattern then return data for the first.
 * @param myPort the bit pattern for port to check
 * @return the number of bytes available for writing in the tx buffer.
 ****************************************************************************/
int wtmSerialX::sxAvailableForWrite(uint8_t myPort)
{
    int afw = 0;

    if (myPort & SX_UART_A)
    {
        afw = uartA->availableForWrite();
        if (afw > 0)
        {
            return afw;
        }
    }
    
    if (myPort & SX_UART_B) 
    {
        afw = uartB->availableForWrite();
        if (afw > 0)
        {
            return afw;
        }
    }
    
    if (myPort & SX_UART_C) 
    {
        afw = uartC->availableForWrite();
        if (afw > 0)
        {
            return afw;
        }
    }
    
    if (myPort & SX_TELNET_PORTS) 
    {
#ifdef ESP32_TYPE_MCU
        afw = TELNET_AVAILABLE_FOR_WRITE;
#else
        // determine maximum output size "fair TCP use"
        // client.availableForWrite() returns 0 when !client.connected()
        int maxToTcp = 0;
        
        for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
        {
            if ( myPort & (SX_TELNET_A << i)) 
            {
                if (serverClients[i].connected())
                {
                    int afw = serverClients[i].availableForWrite();
                    if (afw) 
                    {
                        if (!maxToTcp) {
                            maxToTcp = afw;
                        } else {
                            maxToTcp = std::min(maxToTcp, afw);
                        }
                    } else {
                        // warn but ignore congested clients
                        DBG("Client [%d] is congested\r\n", i);
                    }
                }
            }
        }
        if (maxToTcp > 0)
        {
            return maxToTcp;
        }
#endif
    }

    // If we get here it means there is no available data to be read
    return afw;
}

/****************************************************************************
 * Wait until the Rx Fifo of the specified port has the number of bytes 
 * requested. Used by sxReadBytes().
 * @param myPort the bit pattern for port or ports to check
 * @param sizeToRead the number of bytes requested to wait for
 * @param msTimeout timeout in ms
 * @return the number of bytes available to read.
 ****************************************************************************/

int wtmSerialX::sxWaitUntilNBytesAvailable(uint8_t myPort, size_t sizeToRead, uint64_t msTimeout)
{
    uint64_t msStartTime = millis();
    uint64_t msTimeElapsed = 0;
    size_t afr = 0;
    bool bTimeout = false;

    do 
    {
        afr = sxAvailable(myPort);
        if (afr >= sizeToRead)
        {
            bTimeout = true;
        } else {
            msTimeElapsed = ((uint64_t)millis() - msStartTime);
            if  (msTimeElapsed >= msTimeout)
            {
                bTimeout = true;
            } else {
                delay(10);
            }
        }
    } while (bTimeout==false);

    //DBG("Time elapsed = %llu ms\r\n", msTimeElapsed);
    return int(afr);
}

/****************************************************************************
 * Parse the read data buffer for telnet control characters and remove them. 
 * In addition, data parsed with byte values the same as telnet control chars
 * will be encoded with an extra byte and this extra byte also needs to 
 * be removed from the buffer. Called by sxReadBytes().
 * 
 * xmodem will set fileTransferMode to true. If this is detected the function 
 * specifically reads additional bytes to replace those deleted.
 * 
 * @param clientNum is the specific number of the client in the array of clients.
 * @param buffer the buffer on which to parse
 * @param dataSize the number of bytes to parse
 * @return the number of valid bytes in the buffer after parsing.
 ****************************************************************************/
int wtmSerialX::sxParseTelnetControlChars(int clientNum, char* buffer, int dataSize)
{

    // Simple code to remove extra telnet characters
    // Would be nice to implement telnet protocol fully

    bool curPosFFFF = false;
    bool prevPosFFFF = false;

    // Need at least two bytes to process Telnet control characters
    if (dataSize < 2) return dataSize;

    for (int bufPos = 1; bufPos < dataSize; bufPos++)
    {
        prevPosFFFF = curPosFFFF;
        curPosFFFF = false;
        // In telnet mode every 0xff is replaced by 0xff 0xff
        // If not in binary mode every 0x0d is replaced by 0x0d 0x00
        if ( ( (buffer[bufPos] == 0xff) && (buffer[bufPos - 1] == 0xff) )
            ||  ( ( (_telnetBinaryMode & _myPort) == 0) 
                && (buffer[bufPos] == 0x00) 
                && (buffer[bufPos - 1] == 0x0d) ) )
        {
            if (buffer[bufPos] == 0xff)
            {
                curPosFFFF = true;
            }

#ifdef SX_TEL_RX_DEBUG
            logger->printf("Telnet extra char bufpos = 0x%02x, value = 0x%02x, value before = 0x%02x\r\n", bufPos, buffer[bufPos], buffer[bufPos - 1]);
            logger->printf("_telnetBinaryMode = %d\r\n", _telnetBinaryMode);
#endif
            for (int bufPos2 = bufPos; bufPos2 < dataSize - 1; bufPos2++)
            {
                buffer[bufPos2] = buffer[bufPos2 + 1];
            }
            if (fileTransferMode == true)
            {
                uint8_t byteRead = 0;
#ifdef SX_TEL_RX_DEBUG
                uint8_t addBytesRead = serverClients[clientNum].readBytes(&byteRead, 1);
                logger->printf("Read extra byte = 0x%02x, addBytesRead = %d\r\n", byteRead, addBytesRead);
#else
                serverClients[clientNum].readBytes(&byteRead, 1);
#endif
                buffer[dataSize - 1] = byteRead;
            } else {
                dataSize = dataSize - 1;
            }
        } else {
            // Check for a known telnet command byte in the data
            if ( (buffer[bufPos - 1] == 0xff) && ((bufPos + 1) < dataSize)
                && (prevPosFFFF == false)
                && ( (buffer[bufPos] > 0xEF) &&  (buffer[bufPos] < 0xff) ) )
            {

                // External call to decode and log the command bytes
                logTelnetCommandBytes(true, &buffer[bufPos-1]);

                // If command is IAC-WILL-BINARY TRANSMISSION set binary mode)
                // Todo : Would be better to handle commands in their own function.
                //        That way we could reply with the correct response.
                if ( (buffer[bufPos] == 0xFB) && (buffer[bufPos + 1] == 0) )
                {
#ifdef SX_TEL_RX_DEBUG
                    logger->printf("Previous binary mode = 0x%02x\r\n", _telnetBinaryMode);
                    _telnetBinaryMode |= (_myPort & (SX_TELNET_PORTS << clientNum));
                    logger->printf("New binary mode = 0x%02x\r\n", _telnetBinaryMode);
#else
                    _telnetBinaryMode |= (_myPort & (SX_TELNET_PORTS << clientNum));
#endif
                }
                // Remove the Telnet command bytes from the Rx data.
                // If there is valid data after the command bytes delete the three bytes and move everything left.
                // If the command bytes are at the end of the message then just delete them.
#ifdef SX_TEL_RX_DEBUG
                logger->printf("Delete the three telnet command bytes just received from Rx Data\r\n");
#endif
                if (dataSize > bufPos + 2)
                {
                    for (int newBufPos = bufPos - 1; newBufPos < dataSize - 3; newBufPos++)
                    {
#ifdef SX_TEL_RX_DEBUG
                        logger->printf("Pos %d : replace 0x%02x with 0x%02x\r\n", newBufPos, buffer[newBufPos], buffer[newBufPos + 3]);
#endif
                        buffer[newBufPos] = buffer[newBufPos + 3];
                    }
                    dataSize = dataSize - 3;
                    if (bufPos > 0) bufPos = bufPos - 1;
                } else {
                    dataSize = dataSize - 3;
                }
            }
        }
    }

    return dataSize;
}

/****************************************************************************
 * Read a specific number of bytes from the first port in a specific bit 
 * pattern. Ideally there will only be one bit set but there may be more.
 * _readBytesPortId records which port bit in the bit pattern we read from.
 * @param myPort the bit pattern for port or ports to read from
 * @param buffer the buffer to put the read data
 * @param size the number of bytes to read
 * @return the number of bytes read
 ****************************************************************************/
size_t wtmSerialX::sxReadBytes(uint8_t myPort, char* buffer, size_t size)
{
    size_t sizeRead = 0;
    bool boolNoValidData = true;

    // Check if there are enough bytes available to read. If not wait until there are, or timeout.
    _readBytesPortId = 0;
    size_t afr = sxWaitUntilNBytesAvailable(myPort, size, SX_READ_TIMEOUT);
    if (afr < size)
    {
        DBG("Mismatch in bytes. Bytes requested = %d, available bytes = %d\r\n", size, afr);
        size = afr;
    }

    if (myPort & SX_UART_A)
    {
        sizeRead = uartA->readBytes(buffer, size);
        if (sizeRead > 0) 
        {
            boolNoValidData = false;
            _readBytesPortId = SX_UART_A;
            return sizeRead;
        }
    } 
    
    if ( (myPort & SX_UART_B) && boolNoValidData ) 
    {
#ifndef ESP32_TYPE_MCU
        if (uartB->isRxEnabled())
        {
#endif
            sizeRead = uartB->readBytes(buffer, size);
            if (sizeRead > 0) 
            {
                boolNoValidData = false;
                _readBytesPortId = SX_UART_B;
                return sizeRead;
            }
#ifndef ESP32_TYPE_MCU
        } else {
            DBG("UARTB RX is not enabled");
        }
#endif
    }
    
    if ( (myPort & SX_UART_C) && boolNoValidData ) 
    {
        sizeRead = uartC->readBytes(buffer, size);
        if (sizeRead > 0) 
        {
            boolNoValidData = false;
            _readBytesPortId = SX_UART_C;
            return sizeRead;
        }
    }
    
    if ( (myPort & SX_TELNET_PORTS) && boolNoValidData )
    {
        for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
        {
            if (  (myPort & (SX_TELNET_A << i)) && serverClients[i].connected() && boolNoValidData ) 
            {
                sizeRead = (size_t)serverClients[i].readBytes(buffer, size);

#ifdef SX_TEL_RX_DEBUG
                logger->printf("Dump packet received.\r\n");
                dumpPacket(buffer, sizeRead);
#endif
                if (sizeRead > 0) 
                {
                    sizeRead = sxParseTelnetControlChars(i, buffer, sizeRead);
                    boolNoValidData = false;
                    _readBytesPortId = SX_TELNET_A << i;
                    // Todo : This could do with improving or get rid off
                    // Need to get rid of CR+LF and any other extra characters if expecting just one char
                    // so clear the read buffer if in this mode
                    if (oneByteOnlyMode == true)
                    {
                        while (serverClients[i].available() > 0)
                        {
                            char dummyRead;
                            serverClients[i].readBytes(&dummyRead, 1);
                        }
                    }
                    return sizeRead;
                }

            }
        }
    }

    return sizeRead;
}
