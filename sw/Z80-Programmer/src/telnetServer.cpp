/*************************************************************************
 * 
 *    Code by Denno Wiggle aka WTM
 * 
 *    This is code to host a telnet server with telnet clients. 
 * 
 *    This is a very simple telnet server and does basic client configuration
 *    on setup.
 *    
 *    Two telnet ports are supported. It should be possible to increase 
 *    the number of telnet ports fairly easily.
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
#include "defines.h"

#ifdef ESP32_TYPE_MCU
#include <WiFiClient.h>
#include <WebServer.h>
#include <HTTPClient.h>
#endif /* ESP32_TYPE_MCU */

#ifdef ESP2866_TYPE_MCU
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#endif /* ESP2866_TYPE_MCU */ 

#include "wtmSerialx.h"
#include "dbg.h"
#include "telnetServer.h"

// External wtmSerialx variables on which actions happen when telnet clients 
// connect and disconnect. Console is the only port where telnet is added.
// Within console is where z80uart is connected. 
extern wtmSerialX* console;
extern wtmSerialX* z80uart;
extern wtmSerialX* logger;
extern wtmSerialX* xmodemUart;

// Declare the telnet server and its clients
WiFiServer telnetServer(TELNET_PORT_NUM_CONSOLE);
WiFiServer telnetServerZ80(TELNET_PORT_NUM_Z80);
WiFiClient serverClients[MAX_TELNET_CLIENTS];

// Holds the status of each of the client connections that are in use.
wtmTelnetClientStatus clientStatus[MAX_TELNET_CLIENTS];


/****************************************************************************
 * Function to swap uart port A with port C on the ESP2866. Both A and C
 * use the same internal Serial 0 port on the ESP2866, but on different pins.
 * The ESP2866 only has two uart ports. Uart port B (1) only has TX pin 
 * available.
 * @param enableUartC = True connects the pins for the z80 uart to Serial 0.
 * When set to false the pins for the external serial console (USB) are 
 * connected to Serial 0.
*****************************************************************************/
#ifdef ESP2866_TYPE_MCU
void swapUartAUartC(bool enableUartC)
{
    logger->printf("z/o-1: z80uart->boolUartASwapped = %s\r\n", z80uart->boolUartASwapped?"YES":"NO");
    if (enableUartC)
    {
        // only swap UARTA if it has not been swapped before on ESP2866
        if (!z80uart->boolUartASwapped) 
        {
            z80uart->boolUartASwapped = true;
            // Disable console on UARTA
            console->printf("Switching uart ports to Z80 retro computer\r\n");
            console->printf("Console now locked on this screen\r\n");
            console->flush();
            console->disablePort(SX_UART_A);
            uartA->end();
            uartC->begin(z80uart->baudRate);
            // Note: swap must come after begin
            uartC->swap();
            logger->printf("z80uart baud = %d\r\n", uartC->baudRate());
            uartC->setRxBufferSize(Z80_RX_BUF_SIZE);
            // Flush the Rx buffer
            while(uartC->available())
            {
                uartC->read();
            }
        }
    } else {
        if ( (z80uart->portCfg() == 0) && (z80uart->boolUartASwapped) )
        {
            uartC->swap();
            uartC->end();
            uartA->begin(console->baudRate);
            z80uart->boolUartASwapped = false;
            uartA->setRxBufferSize(CONSOLE_RX_BUF_SIZE);
            // Flush the Rx buffer
            while(uartA->available())
            {
                uartA->read();
            }
            // Reconnect console
            // Enable console on UARTA
            console->enablePort(SX_UART_A);
            console->printf("This console screen is now unlocked\r\n");
        }
    }
    logger->printf("z/o-2: z80uart->boolUartASwapped = %s\r\n", z80uart->boolUartASwapped?"YES":"NO");
}
#endif

/****************************************************************************
 * Function to see if a client number used in serverClients(clientNum) is valid.
 * For future use.
 * @param clientNum is the specific number of the client in the array of clients.
 * @return true if it is a valid client.
*****************************************************************************/
bool clientNumValid(int clientNum)
{
    if ( (clientNum >= 0) && (clientNum < MAX_TELNET_CLIENTS) )
    {
        // Todo : Might want to check that the client is still connected?
        //        Unless this function is being called before doing that.
        return true;
    } else {
        return false;
    }
}

/****************************************************************************
 * Function to start the telnet server on code start-up.
 * The WiFi needs to be configure before this.
*****************************************************************************/
void startTelnetServer()
{
    console->printf("Starting Telnet Server\r\n");
    telnetServer.begin();
    telnetServerZ80.begin();
    
    // nodelay set to true to disable Nagle algorithm.
    // The Nagle algorithm is intended to reduce TCP/IP traffic of small packets sent over the network by
    // combining a number of small outgoing messages, and sending them all at once. The downside of such
    // approach is effectively delaying individual messages until a big enough packet is assembled.
    telnetServer.setNoDelay(true);
    telnetServerZ80.setNoDelay(true);
    // Timeout set to one second. So we if try and read when nothing is in the buffer we will wait 
    // one second for some bytes and then time oout if nothing is received.
    serverClients->setTimeout(1000);
    
#ifndef ESP32_TYPE_MCU
    // This is an experimental API that will set the client in synchronized mode. In this mode, every write
    // () is flushed. It means that after a call to write(), data are ensured to be received where they 
    // went sent to (that is flush semantic).
    // When set to true in WiFiClient implementation,
    // * It slows down transfers, and implicitly disable the Nagle algorithm.
    // * It also allows to avoid a temporary copy of data that otherwise consumes at most TCP_SND_BUF = (2 * MSS) bytes per connection,
    serverClients->setSync(false);
#endif

    for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
    {
        clientStatus[i].connected = false;
        clientStatus[i].binaryMode = false;
        clientStatus[i].port = 0;
    }

    if (WiFi.getMode() == WIFI_AP) 
    {
        console->printf("Use 'telnet %s:%d' to connect to Z80.\r\n", WiFi.softAPIP().toString().c_str(), TELNET_PORT_NUM_Z80);
        console->printf("Use 'telnet %s:%d' to connect to Programmer Console.\r\n\n", WiFi.softAPIP().toString().c_str(), TELNET_PORT_NUM_CONSOLE);
    } else {
        console->printf("Use 'telnet %s:%d' to connect to Z80.\r\n", WiFi.localIP().toString().c_str(), TELNET_PORT_NUM_Z80);
        console->printf("Use 'telnet %s:%d' to connect to Programmer Console.\r\n\n", WiFi.localIP().toString().c_str(), TELNET_PORT_NUM_CONSOLE);
    }
}

/****************************************************************************
 * logTelnetCommandBytes() is used for debug to decode the command bytes
 * that are used in telnet
 * @param fromClient is true when Rx from client, false if Tx from server
 * @param cmd[] holds the three byte command bytes
 * @return true if the command could be decoded, false if not
*****************************************************************************/
bool logTelnetCommandBytes(bool fromClient, char cmd[])
{
    int arrSize;
    uint8_t byteNum;
    bool bFoundCmd = false;
    bool bFoundCmdOption = false;

    if (cmd[0] < 255) 
    {
        return false;
    }

    if (fromClient)
    {
        logger->printf("Telnet command received from client = {%d, %d, %d}\r\n", cmd[0], cmd[1], cmd[2]);
        logger->printf("  RX: ");
    } else {
        logger->printf("Telnet command sent to client       = {%d, %d, %d}\r\n", cmd[0], cmd[1], cmd[2]);
        logger->printf("  TX: ");
    }
     
    // Decode the first two command bytes
    arrSize = sizeof(telCmd) / sizeof(telCmd[0]);
    for (byteNum = 0; byteNum < 2; byteNum++)
    {
        for (int i = 0; i < arrSize; i++)
        {
            if (telCmd[i].code == cmd[byteNum])
            {
                logger->printf("%s-", telCmd[ i ].message);
                bFoundCmd = true;
                break;
            }
        }
        if (bFoundCmd == false)
        {
            logger->printf("UNKOWN COMMAND (%d) ", cmd[byteNum]);
        }
    }

    // Decode the third "option" byte 
    arrSize = sizeof(telCmdOption) / sizeof(telCmdOption[0]);
    for (int i = 0; i < arrSize; i++)
    {
        if (telCmdOption[i].code == cmd[2])
        {
            logger->printf("%s ", telCmdOption[ i ].message);
            bFoundCmdOption = true;
            break;
        }
    }
    if (bFoundCmdOption == false)
    {
        logger->printf("UNKOWN OPTION (%d) ", cmd[2]);
    }

    logger->printf("\r\n");

    return true;
}


/****************************************************************************
 * Function to read the three telnet command bytes.
 * At present we just log them here as part of monitoring client setup.
 * @param clientNum is the specific number of the client in the array of clients.
 * @param msgRx is expected to be a three byte telnet command message
*****************************************************************************/
void readTelnetCommandBytes(int clientNum, char msgRx[])
{
    logger->printf("Client Command Bytes avaiable = %d\r\n", serverClients[clientNum].available());
    while (serverClients[clientNum].available() > 2)
    {
        serverClients[clientNum].readBytes(msgRx, 3);
        if (msgRx[0] == 255) 
        {
            logTelnetCommandBytes(true, msgRx);
        }
    }
}

/****************************************************************************
 * Function to monitor the status of the telnet server and client sessions
 * On new connections command setup messages are sent to the client terminal, 
 * and then wtmSerialx ports are set up appropriately.
 * On closed connections the specific client is removed from the wtmSerialx 
 * port.
 * At present wtmSerialx variable 'console' is the only one configured 
 * for telnet. Adding telnet clients to 'logger' could cause a recursive loop
 * to occur when trying to print telnet debug log messages.
*****************************************************************************/
void handleTelnet()
{
    bool consoleHasClient = telnetServer.hasClient();
    bool z80HasClient = telnetServerZ80.hasClient();
    // Check to see if there are any new clients
    if (consoleHasClient || z80HasClient) 
    {
        // Find the first free location to attach the client
        int i;
        for (i = 0; i < MAX_TELNET_CLIENTS; i++)
        {
            if (!serverClients[i].connected())
            {
                if (consoleHasClient)
                {
                    serverClients[i] = telnetServer.accept();
                    clientStatus[i].port = TELNET_PORT_NUM_CONSOLE;
                } else {
                    serverClients[i] = telnetServerZ80.accept();
                    clientStatus[i].port = TELNET_PORT_NUM_Z80;
                }
                logger->printf("New client at IP address %s connected: client %d port %d\r\n", serverClients[i].remoteIP().toString().c_str(), i, clientStatus[i].port);
                break;
            }
        }
        
        // If there are no avaiable locations to attach the client
        // then send the client a busy busy message and log 
        // that we are busy with active connections.
        if (i == MAX_TELNET_CLIENTS) {
            if (consoleHasClient)
            {
                telnetServer.accept().println("Z80 Retro ESP Programmer is busy.");
            } else {
                telnetServerZ80.accept().println("Z80 Retro ESP Programmer is busy.");
            }
            // hints: server.available() is a WiFiClient with short-term scope
            // when out of scope, a WiFiClient will
            // - flush() - all data will be sent
            // - stop() - automatically
            logger->printf("server is busy with %d active connections\r\n", MAX_TELNET_CLIENTS);
        }
    }

    // Check for new connections and check for disconnections
    for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
    {
        if (serverClients[i].connected())
        {
            if (clientStatus[i].connected == false)
            {
                // New Connection to setup. 
                // First configure the terminal into the mode supported by this program 
                // using telnet command messages.

                clientStatus[i].connected = true;
                serverClients[i].setNoDelay(true);

                // Server should force character mode (no return key needed on client)
                // Command codes taken from here
                // https://www.geeksforgeeks.org/introduction-to-telnet/
                delay(100);
                // Clear any message that might be present
                // Todo : is this really needed
                char msgRx[3] = {0, 0, 0};
                readTelnetCommandBytes(i, msgRx);

                // Send 'Do - Suppress go ahead'
                // 255 : IAC command byte | 253 : Do    |  3 : Suppress go ahead
                char msgTx[3] = {255, 253, 3};
                logTelnetCommandBytes(false, msgTx);
                serverClients[i].write(msgTx, 3);
                delay(100);
                readTelnetCommandBytes(i, msgRx);

                // Note : 'Will - Echo' = works. 'Do - Echo' = not working. 

                // Send 'Do - Echo'
                // 255 : IAC command byte | 253 : Do    |  1 : Echo
                // msg[1] = 253;
                // msg[2] = 1;
                // logTelnetCommandBytes(false, msg);
                // serverClients[i].write(msg, 3);
                // delay(100);
                // readTelnetCommandBytes(i);

                // Send 'Will - Echo'
                // 255 : IAC command byte | 251 : Will  |  1 : Echo
                msgTx[1] = 251;
                msgTx[2] = 1;
                logTelnetCommandBytes(false, msgTx);
                serverClients[i].write(msgTx, 3);
                delay(100);
                readTelnetCommandBytes(i, msgRx);

                // Send 'Don't - Line mode' 
                // CR/LF will not be required to send a byte.
                // This puts the terminal in character mode
                // 255 : IAC command byte | 254 : Don't | 34 : Line mode
                msgTx[1] = 254;
                msgTx[2] = 34;
                logTelnetCommandBytes(false, msgTx);
                serverClients[i].write(msgTx, 3);
                delay(100);
                readTelnetCommandBytes(i, msgRx);

                // Binary mode if used will not add a 0x00 after 0x0d. 
                // 0xff after 0xff will still be added

                // Send 'Do - Binary mode' 
                // 255 : IAC command byte | 253 : Do    |  0 : Binary mode
                msgTx[1] = 253;
                msgTx[2] = 0;
                logTelnetCommandBytes(false, msgTx);
                serverClients[i].write(msgTx, 3);
                delay(100);
                readTelnetCommandBytes(i, msgRx);

                // Check Rx message is 'Will - Binary mode' confirming acceptance.
                // We then remember this setting on the console port
                if ( (msgRx[0] == 255) && ((msgRx[1] == 251) || (msgRx[1] == 253)) && (msgRx[2] == 0) )
                {
                    clientStatus[i].binaryMode = true;
                    console->setTelnetBinaryMode(SX_TELNET_A << i);
                    z80uart->setTelnetBinaryMode(SX_TELNET_A << i);
                    xmodemUart->setTelnetBinaryMode(SX_TELNET_A << i);
                }

                // Enable reading and printing from these interfaces
                // Print a welcome message ONLY on the new telnet connection
                logger->printf("clientStatus[i].port = %d\r\n", clientStatus[i].port);
                if (clientStatus[i].port == TELNET_PORT_NUM_CONSOLE)
                {
                    uint8_t portCfg = console->portCfg();
                    console->disablePort(portCfg);
                    console->enablePort(SX_TELNET_A << i);
                    console->printf("The telnet session is connected to the ESP Console:\r\n");
                    console->enablePort(portCfg);
                    logger->printf("Telnet Welcome : console->portCfg() = 0x%02x\r\n", console->portCfg());
                }
                if (clientStatus[i].port== TELNET_PORT_NUM_Z80)
                {
#ifdef ESP2866_TYPE_MCU
                    swapUartAUartC(true);
#endif
                    uint8_t portCfg = z80uart->portCfg();
                    z80uart->disablePort(portCfg);
                    z80uart->enablePort(SX_TELNET_A << i);
                    z80uart->printf("The telnet session is connected to the Z80 console:\r\n");
                    z80uart->enablePort(portCfg);
                    logger->printf("Telnet Welcome : z80uart->portCfg() = 0x%02x\r\n", z80uart->portCfg());
                }
            }
        } else {
            if (clientStatus[i].connected)
            {
                logger->printf("Client %d disconnected.\r\n", i);
                clientStatus[i].connected = false;
                clientStatus[i].binaryMode = false;
                console->unsetTelnetBinaryMode( (SX_TELNET_A << i) );
                z80uart->unsetTelnetBinaryMode( (SX_TELNET_A << i) );
                xmodemUart->unsetTelnetBinaryMode( (SX_TELNET_A << i) );

                // Disable reading and printing from all interfaces
                if (clientStatus[i].port == TELNET_PORT_NUM_CONSOLE)
                {
                    uint8_t portCfg = (SX_TELNET_A << i);
                    console->disablePort(portCfg);
                    logger->disablePort(portCfg);
                }
                if (clientStatus[i].port == TELNET_PORT_NUM_Z80)
                {
                    uint8_t portCfg = (SX_TELNET_A << i);
                    z80uart->disablePort(portCfg);
#ifdef ESP2866_TYPE_MCU
                    swapUartAUartC(false);
#endif
                }
                clientStatus[i].port = 0;
            }
        }
    }
}

