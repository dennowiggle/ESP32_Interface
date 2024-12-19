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

#ifndef __WTM_TELNET_SERVER__
#define __WTM_TELNET_SERVER__

#include <stdint.h>
#include <stdbool.h> 
#include <Arduino.h>


/****************************************************************************
 * Structure type definition to maintain telnet status specific to the 
 * client that connected.
 * @param connected is a variable to store the client connect status. This is 
 * a status variable that is used in status change checks to see if setup and 
 * tear down tasks need to be implemented on the wtmSerialx ports.
 * @param binaryMode is used to store if the client is connected in binary 
 * mode. The variable is currently redundant (not used) but kept to aid 
 * in debugging with any future issues.
 * @param port stores the port number that the client connect with. This is 
 * used to determine which user terminal to attach the user to. 
 ****************************************************************************/
struct TelnetClientStatus {
    bool     connected = false;
    bool     binaryMode = false;
    uint16_t port = 0;
} typedef wtmTelnetClientStatus;


/****************************************************************************
 * Structure type definition to help decode telnet commands (byte 2 of 3) 
 * into a string. Used to decode messages.
 * @param code is byte code for the telnet command byte.
 * @param message is a text string of the command byte.
*****************************************************************************/
struct telnetCmd {
    int  code;
    const char *message;
} typedef wtmTelnetCmd;


/****************************************************************************
 * Array telCmd holds the possible telnet commands.
 * ref: https://en.wikipedia.org/wiki/Telnet
 * ref: https://www.omnisecu.com/tcpip/telnet-commands-and-options.php
 * ref: https://www.ibm.com/docs/en/zvm/7.2?topic=names-telnet
 * *****************************************************************************/
static wtmTelnetCmd telCmd[] =
{
    // 0-239, 255, "UNASSIGNED"
    // End of subnegotiation parameters.
    { 240, "SE" },
    // No operation. 
    { 241, "NOP" }, 
    // Data Mark (part of the Synch function). Indicates the position of a Synch event within the data stream. 
    { 242, "DATA MARK" }, 
    // NYT character break. 
    { 243, "BREAK" }, 
    // Suspend, interrupt or abort the process to which the NVT is connected.
    { 244, "INTERRUPT PROCESS" }, 
    // Abort Output. Allows the current process to run to completion but do not send its output to the user. 
    { 245, "ABORT OUTPUT" }, 
    // Are You There is used to determine if the remote TELNET partner is still up and running. 
    { 246, "ARE YOU THERE?" }, 
    // Erase character is used to indicate the receiver should delete the last preceding undeleted character from the data stream.
    { 247, "ERASE CHARACTER" }, 
    // Delete characters from the data stream back to but not including the previous CRLF.
    { 248, "ERASE LINE" }, 
    // Go ahead is used in half-duplex mode to indicate the other end that it can transmit.
    { 249, "GO AHEAD" },
    // Begin of subnegotiation
    { 250, "SB" },
    // Will. The sender wants to enable an option
    { 251, "WILL" },
    // Won't. The sender do not wants to enable an option.
    { 252, "WON'T" },
    // Do. Sender asks receiver to enable an option.
    { 253, "DO" },
    // Don't. Sender asks receiver not to enable an option.
    { 254, "DON'T" },
    // IAC Command byte
    { 255, "IAC" },
};

/****************************************************************************
 * Structure type definition to help decode telnet command options (byte 3 
 * of 3) into a string. Used to decode messages.
 * @param code is byte code for the telnet command option byte.
 * @param message is a text string of the command byte.
*****************************************************************************/
struct telnetCmdOption {
    int  code;
    const char *message;
} typedef wtmTelnetCmdOption;


/****************************************************************************
 * Array holds the possible telnet command options.
 * ref: https://en.wikipedia.org/wiki/Telnet
 * ref: https://www.omnisecu.com/tcpip/telnet-commands-and-options.php
 * ref: https://www.ibm.com/docs/en/zvm/7.2?topic=names-telnet
 * *****************************************************************************/
static wtmTelnetCmdOption telCmdOption[] = 
{
    { 0, "BINARY TRANSMISSION" },
    { 1, "ECHO" },
    { 2, "RECONNECTION" },
    { 3, "SUPPRESS GO AHEAD" },
    { 4, "APPROX MESSAGE SIZE NEGOTIATION" },
    { 5, "STATUS" },
    { 6, "TIMING MARK" },
    { 7, "REMOTE CONTROLLED TRANS AND ECHO" },
    { 8, "OUTPUT LINE WIDTH" },
    { 9, "OUTPUT PAGE SIZE" },
    { 10, "OUTPUT CARRIAGE-RETURN DISPOSITION" },
    { 11, "OUTPUT HORIZONTAL TAB STOPS" },
    { 12, "OUTPUT HORIZONTAL TAB DISPOSITION" },
    { 13, "OUTPUT FORMFEED DISPOSITION" },
    { 14, "OUTPUT VERTICAL TABSTOPS" },
    { 15, "OUTPUT VERTICAL TAB DISPOSITION" },
    { 16, "OUTPUT LINEFEED DISPOSITION" },
    { 17, "EXTENDED ASCII" },
    { 18, "LOGOUT" },
    { 19, "BYTE MACRO" },
    { 20, "DATA ENTRY TERMINAL" },
    { 22, "SUPDUP OUTPUT" },
    { 23, "SEND LOCATION" },
    { 24, "TERMINAL TYPE" },  // Example asking to send terminal type IAC (255) SB (250) TERMINAL-TYPE (24) SEND (0x01) IAC (255) SE (240)
                              // https://www.rfc-editor.org/rfc/rfc930
    { 25, "END OF RECORD" },
    { 26, "TACACS USER IDENTIFICATION" },
    { 27, "OUTPUT MARKING" },
    { 28, "TERMINAL LOCATION NUMBER" },
    { 29, "TELNET 3270 REGIME" },
    { 30, "X.3 PAD" },
    { 31, "NEGOTIATE ABOUT WINDOW SIZE" },
    { 32, "TERMINAL SPEED" },
    { 33, "REMOTE FLOW CONTROL" },
    { 34, "LINEMODE" },
    { 35, "X DISPLAY LOCATION" },
    { 36, "ENVIRONMENT OPTION" },
    { 37, "AUTHENTICATION OPTION" },
    { 38, "ENCRYPTION OPTION" },
    { 39, "NEW ENVIRONMENT OPTION" },
    { 40, "TN3270E" },
    { 41, "XAUTH" },
    { 42, "CHARSET" },
    { 43, "TELNET REMOTE SERIAL PORT" },
    { 44, "COM PORT CONTROL OPTION" },
    { 45, "TELNET SUPPRESS LOCAL ECHO" },
    { 46, "TELNET START TLS" },
    { 47, "KERMIT" },
    { 48, "SEND-URL" },
    { 49, "FORWARD_X" },
    // 50-137, "UNASSIGNED"
    { 138, "TELOPT PRAGMA LOGON" },
    { 139, "TELOPT SSPI LOGON" },
    { 140, "TELOPT PRAGMA HEARTBEAT" },
    // 141-254, "UNASSIGNED"
    { 255, "EXTENDED-OPTIONS-LIST" },
};


/****************************************************************************
 * Function to swap uart port A with port C on the ESP2866. Both A and C
 * use the same internal Serial 0 port on the ESP2866, but on different pins.
 * The ESP2866 only has two uart ports. Uart port B (1) only has the TX pin 
 * available.
 * @param enableUartC = True connects the pins for the z80 uart to Serial 0.
 * When set to false the pins for the external serial console (USB) are 
 * connected to Serial 0.
*****************************************************************************/
#ifdef ESP2866_TYPE_MCU
void swapUartAUartC(bool enableUartC);
#endif

/****************************************************************************
 * logTelnetCommandBytes() is used for debug to decode the command bytes
 * that are used in telnet
 * @param fromClient is true when Rx from client, false if Tx from server
 * @param cmd[] holds the three byte command bytes
 * @return true if the command could be decoded, false if not
*****************************************************************************/
bool logTelnetCommandBytes(bool fromClient, char cmd[]);


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
void handleTelnet();


/****************************************************************************
 * Function to start the telnet server on code start-up.
 * The WiFi needs to be configure before this.
*****************************************************************************/
void startTelnetServer();


#endif /* __WTM_TELNET_SERVER__ */