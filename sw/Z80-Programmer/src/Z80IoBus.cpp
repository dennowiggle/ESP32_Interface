/*************************************************************************
 * 
 *    Code & ESP Programmer Board by Denno Wiggle aka WTM
 * 
 *    This is code to control 8 bit message port for communicating
 *    status with the Z80-Retro main board.
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
#include "wtmSerialx.h"
#include "Z80IoBus.h"


extern wtmSerialX* logger;
extern wtmSerialX* console;


void z80InterrupSetup()
{
    pinMode(Z80_INT_N, INPUT);
}

/**
 * @brief Information type for Z80 IORQ information
 * The Z80 IORQ information structure type is used to define the IORQ 
 * address A4 Read and Write status. The detected pins are set by the ISR for
 * the appropriate pin.
 * @param rdPin holds the pin number for the IORQ Read pin.
 * @param wrPin holds the pin number for the IORQ Write pin.
 * @param rdDetected true indicates the rising edge of IORQ Read was detected. 
 * It is set by the read ISR.
 * @param wrDetected true indicates the rising edge of IORQ Write was detected.
 * It is set by the write ISR.
 * @param readCount increments on each IORQ read.
 * @param writeCount increments on each IORQ write.
 *****************************************************************************/
struct wtmIorqInfo {
	const uint8_t rdPin;
	const uint8_t wrPin;
	bool rdDetected;
    bool wrDetected;
    uint32_t readCount;
    uint32_t writeCount;
} typedef wtmIorqInfo;


/**
 * @brief Information type for Z80 IORQ information
 * The Z80 IORQ information structure type is used to define the IORQ 
 * address A4 Read and Write status. The detected pins are set by the ISR for
 * the appropriate pin.
 * @param rdPin holds the pin number for the IORQ Read pin.
 * @param wrPin holds the pin number for the IORQ Write pin.
 * @param rdDetected true indicates the rising edge of IORQ Read was detected. 
 * It is set by the read ISR.
 * @param wrDetected true indicates the rising edge of IORQ Write was detected.
 * It is set by the write ISR.
 * @param readCount increments on each IORQ read.
 * @param writeCount increments on each IORQ write.
 *****************************************************************************/
wtmIorqInfo z80IorqInfo = {
    .rdPin = Z80_IORQ_A0_RD,
    .wrPin = Z80_IORQ_A0_WR,
    .rdDetected = false,
    .wrDetected = false,
    .readCount = 0,
    .writeCount = 0,
};


/**
 * @brief Set up the Z80 message port for reading or writing by the ESP32.
 * @param portSetup defines the pin mode for the pins on this port.
 * e.g. INPUT, OUPUT, INPUT_PULLUP
 * The direction of the board data buffers is set appropriately by the 
 * levell of the Z80_DATA_DIR_W signal.
 *****************************************************************************/
void z80DataPortDirSetup(uint8_t portSetup)
{
    pinMode(Z80_DATA_0, portSetup);
    pinMode(Z80_DATA_1, portSetup);
    pinMode(Z80_DATA_2, portSetup);
    pinMode(Z80_DATA_3, portSetup);
    pinMode(Z80_DATA_4, portSetup);
    pinMode(Z80_DATA_5, portSetup);
    pinMode(Z80_DATA_6, portSetup);
    pinMode(Z80_DATA_7, portSetup);
    pinMode(Z80_DATA_DIR_W, OUTPUT);

    if (portSetup == INPUT || portSetup == INPUT_PULLUP)
    {
        digitalWrite(Z80_DATA_DIR_W, LOW);
    } else {
        digitalWrite(Z80_DATA_DIR_W, HIGH);
    }
}

/**
 * @brief Read a byte from the Z80 message port.
 * @return the byte read from the message port.
 *****************************************************************************/
uint8_t z80DataPortRead()
{
    uint8_t readValue = 0;

    z80DataPortDirSetup(INPUT);

    if (digitalRead(Z80_DATA_0) == HIGH) readValue |= (1<<0);
    if (digitalRead(Z80_DATA_1) == HIGH) readValue |= (1<<1);
    if (digitalRead(Z80_DATA_2) == HIGH) readValue |= (1<<2);
    if (digitalRead(Z80_DATA_3) == HIGH) readValue |= (1<<3);
    if (digitalRead(Z80_DATA_4) == HIGH) readValue |= (1<<4);
    if (digitalRead(Z80_DATA_5) == HIGH) readValue |= (1<<5);
    if (digitalRead(Z80_DATA_6) == HIGH) readValue |= (1<<6);
    if (digitalRead(Z80_DATA_7) == HIGH) readValue |= (1<<7);

    return readValue;
}

/**
 * @brief Write a byte to the Z80 message port.
 * @param data the byte value to write
 *****************************************************************************/
void z80DataPortwrite(uint8_t data)
{
    z80DataPortDirSetup(OUTPUT);

    if (data & (1<<0)) 
    { 
        digitalWrite(Z80_DATA_0, HIGH);
    } else {
        digitalWrite(Z80_DATA_0, LOW);
    }
    if (data & (1<<1)) 
    { 
        digitalWrite(Z80_DATA_1, HIGH);
    } else {
        digitalWrite(Z80_DATA_1, LOW);
    }
    if (data & (1<<2)) 
    { 
        digitalWrite(Z80_DATA_2, HIGH);
    } else {
        digitalWrite(Z80_DATA_2, LOW);
    }
    if (data & (1<<3)) 
    { 
        digitalWrite(Z80_DATA_3, HIGH);
    } else {
        digitalWrite(Z80_DATA_3, LOW);
    }
    if (data & (1<<4)) 
    { 
        digitalWrite(Z80_DATA_4, HIGH);
    } else {
        digitalWrite(Z80_DATA_4, LOW);
    }
    if (data & (1<<5)) 
    { 
        digitalWrite(Z80_DATA_5, HIGH);
    } else {
        digitalWrite(Z80_DATA_5, LOW);
    }
    if (data & (1<<6)) 
    { 
        digitalWrite(Z80_DATA_6, HIGH);
    } else {
        digitalWrite(Z80_DATA_6, LOW);
    }
    if (data & (1<<7)) 
    { 
        digitalWrite(Z80_DATA_7, HIGH);
    } else {
        digitalWrite(Z80_DATA_7, LOW);
    }
}


/**
 * @brief Interrupt service routine for the Z80 IORQ Read signal.
 * When a rising edge is detected on the Z80 IORQ Read signal the detected 
 * status for this signal is set and the trigger count is incremented.
 *****************************************************************************/
void IRAM_ATTR z80IorqRdIsr()
{
    z80IorqInfo.rdDetected = true;
    z80IorqInfo.readCount++;
}

/**
 * @brief Interrupt service routine for the Z80 IORQ Write signal.
 * When a rising edge is detected on the Z80 IORQ Write signal the detected 
 * status for this signal is set and the trigger count is incremented.
 *****************************************************************************/
void IRAM_ATTR z80IorqWrIsr()
{
    z80IorqInfo.wrDetected = true;
    z80IorqInfo.writeCount++;
}

/**
 * @brief Sets up the Z80 IORQ Read and Write pins, and attaches interrupts
 * to them for servicing a rising edge on these signals.
 *****************************************************************************/
void z80IorqSetup()
{
    pinMode(z80IorqInfo.rdPin, INPUT);
    pinMode(z80IorqInfo.wrPin, INPUT);
    attachInterrupt(z80IorqInfo.rdPin, z80IorqRdIsr, RISING);
    attachInterrupt(z80IorqInfo.wrPin, z80IorqWrIsr, RISING);
    logger->printf("Attaching z80IorqRdIsr interrup to pin %d\r\n", z80IorqInfo.rdPin);
    logger->printf("Attaching z80IorqWrIsr interrup to pin %d\r\n", z80IorqInfo.wrPin);
}


/**
 * @brief A handler for servicing the Z80 message byte upon detection of a 
 * read or write by the Z80 Retro main board.
 * 
 * At presnt this just prints debug messages.
 *****************************************************************************/
void handleZ80Iorq()
{
    if (z80IorqInfo.rdDetected)
    {
        z80IorqInfo.rdDetected = false;
        console->printf("Z80 IORQ-read detected! Z80 Read Count = %d.\r\n", z80IorqInfo.readCount);
    }
    if (z80IorqInfo.wrDetected)
    {
        z80IorqInfo.wrDetected = false;
        console->printf("Z80 IORQ-write detected = 0x%02x. Z80 Write Count = %d.\r\n", z80DataPortRead(), z80IorqInfo.writeCount);
    }
}

void handleZ80IorqTest()
{
    static uint8_t count = 0;
    static bool boolStop = false;

    if (z80IorqInfo.rdDetected)
    {
        z80IorqInfo.rdDetected = false;
        uint8_t valRead = z80DataPortRead();
        console->printf("Z80 IORQ-read! Value read = 0x%02x, Count = %d.\r\n", valRead, z80IorqInfo.readCount);
        if (!boolStop) 
        { 
            // Write back 1+ the value read
            console->printf("..... Test Z80 Write = 0x%02x. Test Write Count = %d.\r\n", ++valRead, ++count);
            z80DataPortwrite(valRead);
            if (count == 0) boolStop = true;
        }
    }
    if (z80IorqInfo.wrDetected)
    {
        z80IorqInfo.wrDetected = false;
        console->printf("Z80 IORQ-write detected = 0x%02x. Z80 Write Count = %d.\r\n", z80DataPortRead(), z80IorqInfo.writeCount);
    }
}




