/*****************************************************************************
 * 
 *    Z80-Retro! ESP Programmer
 *    Code & ESP Programmer Board by Denno Wiggle aka WTM
 *    Z80-Retro project by John Winans
 *     - https://github.com/Z80-Retro
 * 
 *    This is code written by John Winans for the 
 *    2065-Z80-programmer project with the following modifications
 *    1. I2C low level communication changed from pi/linux to espressif
 *       ESP2866 / ESP32 code.
 *    2. SPI low level communication option instead of I2C for espressif 
 *       ESP32 CPU.
 *****************************************************************************/


//****************************************************************************
//
//    2065-Z80-programmer I2C FLASH Programmer Application
//
//    Copyright (C) 2021 John Winans
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//
//****************************************************************************

// Raspberry PI application to program the SST39SF010A FLASH
// chip on a 2063-Z80 board by way of using a 2065-Z80-programmer 
// adapter board.
//
// The PI communicates with the 2065-Z80-programmer using I2C.


//#include <errno.h>
#include <Arduino.h>
#include "defines.h"
#ifdef WTM_PROG_I2C
	#include <Wire.h>
#endif
/*
 i2c error codes
0   success
2   received NACK on transmit of address
3   received NACK on transmit of data
4   line busy
*/

#ifdef WTM_PROG_SPI
	#include "SPI.h"
	#include "driver/spi_master.h"
#endif

#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/types.h>
// Linux only
// #include <sys/uio.h>
// #include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/time.h>
#include <string.h>
#include <time.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

// Linux only
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <i2c/smbus.h>

#include <stdint.h>

#include "dbg.h"
#include "defines.h"

// #define I2C_DEV "/dev/i2c-1"

// MCP23017 (expand-o chip) addresses
#ifdef WTM_PROG_I2C
#define IO_EXP1		(0x26)	///< the one connected to the address bus
#define IO_EXP2		(0x24)	///< the one connected to the control and data bus
#endif

#ifdef WTM_PROG_SPI
#define IO_EXP1		(0x4C)	///< the one connected to the address bus
#define IO_EXP2		(0x48)	///< the one connected to the control and data bus
#define IO_EXP_RD	(0x01)	///< Read/Write bit needed for SPI Mode
#endif

// IO_EXP1 A addr [0..7]
// IO_EXP1 B addr [8..15]
// IO_EXP2 A data [0..7]
// IO_EXP2 B bus control [RESET, /RD, /WR, /MREQ, /IORQ, /BUSRQ, /BUSACK, /M1]

// control bus pin assignment on EX2 port B
#define	IO_RESET	(0x01)	///< ALWAYS output
#define	IO_RD_N		(0x02)	///< output when take over the bus, else input/float
#define	IO_WR_N		(0x04)	///< output when take over the bus, else input/float
#define	IO_MREQ_N	(0x08)	///< output when take over the bus, else input/float
#define	IO_IORQ_N	(0x10)	///< output when take over the bus, else input/float
#define	IO_BUSRQ_N	(0x20)	///< ALWAYS output
#define	IO_BUSACK_N	(0x40)	///< ALWAYS input
#define	IO_M1_N		(0x80)	///< ALWAYS input

// These are used to configure the control ports when transitioning from 
// passive to active control mode (ie, when the Z80 bus is taken over by the
// programmer.  Direction control = 1 for inputs, 0 for outputs.
#define IO_DIR_BUS_CTL_ON	(IO_BUSACK_N|IO_M1_N)		// all out (0) except /BUSACK & /M1
#define IO_DIR_BUS_CTL_OFF	(~(IO_RESET|IO_BUSRQ_N))	// /BUSRQ & RESET = out, all others=in


// These are used to set the direction of all 8 bits of a port to in or out
// mode when configuring the address and data lines.
#define IO_DIR_BUS_AD_IN	(0xff)	// all bits = 1 = input
#define IO_DIR_BUS_AD_OUT	(0)		// all bits = 0 = output


/**
* This is used to make the logic that has to toggle one bit (without
* knowing the state of the others) easier to implement.
* It is initialized to the power-on-reset value of the MCP23017 chip.
*****************************************************************************/
static uint8_t ctl_cache = 0xff;		// a copy of the last-written control latch value

/**
* This is a copy of the last-written data-direction seting.
* We use it to figure out if it needs to be changed or not when
* we need to put it into a given direction.
* It is initialized into an impossible value so that the 'first time'
* it is checked, it will indicate that it needs to be set.
*****************************************************************************/
static uint8_t data_dir_cache = 0x0f;


/**
* This is the handle for a spi bus device.
* Needs to be initialised by calling spiCopyHandleForExp()
*****************************************************************************/
#ifdef WTM_PROG_SPI
spi_device_handle_t spiHandle;
bool spiBusAvailable = false;

/**
* Function to copy the spi handle from external source for use to communicate 
* on Z80 bus through the MCP23S17 SPI devices. 
* @param handle is the spi handle for the MCP23S17 devices
*****************************************************************************/
void spiCopyHandleForExp(spi_device_handle_t handle)
{
    spiHandle = handle;
}

/**
* Function to set the availability of the SPI Bus for writing to the
* MCP23S17 devices. This is called by the SPI driver.
* @param available is the spi bus availability.
*****************************************************************************/
void spiSetBusAvailable(bool available)
{
    spiBusAvailable = available;
}

/**
* Function to print the raw SPI TX and RX data
* @param txdata[] is the data that was transmitted to the device
* @param rxdata[] is the data that was received on the bus from the device
* @param len is the number of bits of valid data.
* @param strCallingFunction is an aid to show which function called this.
*****************************************************************************/
#ifdef WTM_DBG_IO_EXPANDER
void spiPrintRawData(uint8_t txData[], uint8_t rxData[], size_t len, const char * strCallingFunction)
{
	if (len > 0)
	{
		//Convert the length in bits to bytes
		len = len / 8;
		console->printf("%s: Transmit (Addr=0x%02x R/Wb=0x%x):", 
		                strCallingFunction, txData[0] & ~IO_EXP_RD, (txData[0] & IO_EXP_RD) );
		for (int index = 0; index<len; index++)
		{
			console->printf(" 0x%02x", txData[index]);
		}
		console->printf("\r\n");

        if (txData[0] & IO_EXP_RD)
		{
			console->printf("%s: Receive  (Addr=0x%02x R/Wb=0x%x):", 
							strCallingFunction, txData[0] & ~IO_EXP_RD, (txData[0] & IO_EXP_RD) );
			for (int index = 0; index<len; index++)
			{
				console->printf(" 0x%02x", rxData[index]);
			}
			console->printf("\r\n");
		}
	}
}
#endif /* WTM_DBG_IO_EXPANDER */

#endif /* WTM_PROG_SPI */

/**
* Decode i2c error codes given by the esp8266 i2c driver
* @param rc is the error code to decode
* returns a string explaining the error code
*****************************************************************************/
char* decodeRc(uint8_t rc)
{
	    switch (rc)
    {
        case 2:
            return (char*)"i2c NACK on transmit of address";
        case 3:
            return (char*)"i2c NACK on transmit of data";
        case 4:
            return (char*)"i2c busy";
        default:
		    static char buf[25];
			sprintf(buf,"error code %d",rc);
            return buf;
    }
}

/**
* Write the given values to ports A and B of the addr-specified MCP23017.
* @param iic not used (possible future use, used to be iic file handle)
* @param addr The iic address of the MCP23017 device to write data into
* @param a The value to write to the GPIO A port register
* @param b The value to write to the GPIO B port register
*****************************************************************************/
int mcp23017_write(int iic, int addr, uint8_t a, uint8_t b)
{
#ifdef WTM_PROG_I2C
    uint8_t rc = 0;

	Wire.beginTransmission(addr);
	Wire.write(0x12);              // register 0x12 = GPIOA
	Wire.write(a);                 // GPIOA value
	Wire.write(b);                 // GPIOB value
	rc = Wire.endTransmission();
    if (rc > 0)
	{
        DBG("write() failed %s\r\n", decodeRc(rc));
		return -1;
	}

#elif defined(WTM_PROG_SPI)

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 4*8;
    t.tx_data[0] = addr;
    t.tx_data[1] = 0x12;           // register 0x12 = GPIOA
    t.tx_data[2] = a;              // GPIOA value
    t.tx_data[3] = b;              // GPIOB value

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("write() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif

#endif
	return 0;
}

/**
* Write the given value to port A of the addr-specified MCP23017.
* @param iic not used (possible future use, used to be iic file handle)
* @param addr The iic address of the MCP23017 device to write data into
* @param a The value to write to the GPIO A port register
*****************************************************************************/
int mcp23017_write_a(int iic, int addr, uint8_t a)
{
#ifdef WTM_PROG_I2C
    uint8_t rc = 0;

	Wire.beginTransmission(addr);
	Wire.write(0x12);              // register 0x12 = GPIOA
	Wire.write(a);                 // GPIOA value
	rc = Wire.endTransmission();
	if(rc > 0)
	{
        DBG("write() failed %s\r\n", decodeRc(rc));
		return -1;
	}

#elif defined(WTM_PROG_SPI)

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 3*8;
    t.tx_data[0] = addr;
    t.tx_data[1] = 0x12;           // register 0x12 = GPIOA
    t.tx_data[2] = a;              // GPIOA value

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("write() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif

#endif

	return 0;
}

/**
* Write the given value to port B of the addr-specified MCP23017.
* @param iic not used (possible future use, used to be iic file handle)
* @param addr The iic address of the MCP23017 device to write data into
* @param b The value to write to the GPIO B port register
*****************************************************************************/
int mcp23017_write_b(int iic, int addr, uint8_t b)
{
#ifdef WTM_PROG_I2C
    uint8_t rc = 0;

	Wire.beginTransmission(addr);
	Wire.write(0x13);              // register 0x13 = GPIOA
	Wire.write(b);                 // GPIOA value
	rc = Wire.endTransmission();
	if(rc > 0)
	{
        DBG("write() failed %s\r\n", decodeRc(rc));
		return -1;
	}

#elif defined(WTM_PROG_SPI)

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 3*8;
    t.tx_data[0] = addr;
    t.tx_data[1] = 0x13;           // register 0x13 = GPIOB
    t.tx_data[2] = b;              // GPIOB value

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("write() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif

#endif

	return 0;

}

/**
* Return 16-bit value from ports B and A (B=msb & A=lsb)
* @param iic not used (possible future use, used to be iic file handle)
* @param addr The iic address of the MCP23017 device to write data into
* @return a 16-bit value comprised of the value read from port A (in the lsb)
*	and the value of port B (in the MSB).
*****************************************************************************/
uint16_t mcp23017_read(int iic, int addr)
{
#ifdef WTM_PROG_I2C
    uint8_t rc = 0;
    uint8_t xbuf[5];
    uint8_t rbuf[5];

	Wire.beginTransmission(addr);
	Wire.write(0x12);              // register 0x12 = GPIOA
	rc = Wire.endTransmission();
	if(rc > 0)
	{
        DBG("write() failed %s\r\n", decodeRc(rc));
		return -1;
	}

	rc = Wire.requestFrom(addr, 2);     // request two bytes
	if(rc != 2)
	{
        DBG("read() failed %d\r\n", rc);
		return -1;
	}
    xbuf[0] = Wire.read();
	xbuf[1] = Wire.read();

	return xbuf[0]|(xbuf[1]<<8);	// A=lsb

#elif defined(WTM_PROG_SPI)

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 4*8;
    t.tx_data[0] = addr | IO_EXP_RD;
    t.tx_data[1] = 0x12;           // register 0x12 = GPIOA
    t.tx_data[2] = 0;              // don't care value
    t.tx_data[3] = 0;              // don't care value

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("read() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif

	return ( t.rx_data[2] | (t.rx_data[3]<<8) );	// A=lsb

#else 
	return -1;
#endif
}

/**
* Set the direction control of each of the pins of ports A and B of the
* MCP23017 at the given address.
* @param iic not used (possible future use, used to be iic file handle)
* @param addr The iic address of the MCP23017 device to set the bit-directions
* @param a An 8-bit value indicating which pins are to be configured for
* output (0) and which for input (1). 
*****************************************************************************/
int mcp23017_set_dir(int iic, uint8_t addr, uint8_t a, uint8_t b)
{
#ifdef WTM_PROG_I2C
    uint8_t rc = 0;
    // rc = ioctl(iic, I2C_SLAVE, addr);

	Wire.beginTransmission(addr);
	Wire.write(0);                 // register 0 = DIRA
	Wire.write(a);                 // DIRA value
	Wire.write(b);                 // DIRB value
	rc = Wire.endTransmission();
	if(rc > 0)
	{
        DBG("write() failed %s\r\n", decodeRc(rc));
		return -1;
	}
	return 0;

#elif defined(WTM_PROG_SPI)

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 4*8;
    t.tx_data[0] = addr;
    t.tx_data[1] = 0x00;           // register 0x00 = DIRA
    t.tx_data[2] = a;              // DIRA value
    t.tx_data[3] = b;              // DIRB value

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("write() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif

	return 0;

#else
	return -1;
#endif
}


/**
* Set the pull-ip control of each of the pins of ports A and B of the
* MCP23017 at the given address.
* @param iic not used (possible future use, used to be iic file handle)
* @param addr The iic address of the MCP23017 device to set the bit-directions
* @param a An 8-bit value indicating which pins are to be configured for
* output (0) and which for input (1). 
*****************************************************************************/
int mcp23017_set_pullup(int iic, uint8_t addr, uint8_t a, uint8_t b)
{
#ifdef WTM_PROG_I2C
    uint8_t rc = 0;

#ifdef WTM_PROG_I2C
	Wire.beginTransmission(addr);
	Wire.write(0x0c);              // register 0xC = pull-up A
	Wire.write(a);                 // pull-up A value
	Wire.write(b);                 // pull-up B value
	rc = Wire.endTransmission();
	if(rc > 0)
	{
        DBG("write() failed %s\r\n", decodeRc(rc));
		return -1;
	}
#elif defined(WTM_PROG_SPI)
#endif
	return 0;

#elif defined(WTM_PROG_SPI)

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 4*8;
    t.tx_data[0] = addr;
    t.tx_data[1] = 0x0c;           // register 0xC = pull-up A
    t.tx_data[2] = a;              // pull-up A value
    t.tx_data[3] = b;              // pull-up B value

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("write() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif
	return 0;

#else 
	return -1;
	
#endif /* WTM_PROG_SPI */
}

#ifdef WTM_PROG_SPI	
/**
* Read the status of the HAEN bit in the IOCON registers 0x0A and 0x0B which 
* in SPI mode controls whether the bus address pins A2, A1, A0 are used.
* @param addr the device bus address of the MCP23S17 device
* @param return the bit 3 setting = HAEN
*****************************************************************************/
uint16_t mcp23017_read_HAEN(int addr)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 4*8;
    t.tx_data[0] = addr | IO_EXP_RD;
    t.tx_data[1] = 0x0A;           //  register 0x05 = HAEN
    t.tx_data[2] = 0;              // don't care value
    t.tx_data[3] = 0;              // don't care value

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("read() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif
    return ((t.rx_data[2] & 0x08) > 0);

}

/**
* Write the HAEN bit in the IOCON registers 0x0A and 0x0B which 
* in SPI mode controls whether the bus address pins A2, A1, A0 are used.
* @param addr the device bus address of the MCP23S17 device
* @param return the bit 3 seeting = HAEN
*****************************************************************************/
uint16_t mcp23017_write_HAEN(int addr)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Try and read the bus values
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 4*8;
    t.tx_data[0] = addr;
    t.tx_data[1] = 0x0A;           // register 0x05 = IOCON 
    t.tx_data[2] = 0x08;           // bit 3, HAEN = 1.
    t.tx_data[3] = 0x08;           // bit 3, HAEN = 1.

    esp_err_t rc = spi_device_transmit(spiHandle, &t);
	if(rc != ESP_OK)
	{
        DBG("read() failed %d\r\n", rc);
		return -1;
	}

#ifdef WTM_DBG_IO_EXPANDER
    spiPrintRawData(t.tx_data, t.rx_data, t.length, __FUNCTION__);
#endif
    return 0;
}
#endif /* WTM_PROG_SPI */

/**
 * @brief Initialise the HAEN bit in the IOCON registers 0x0A and 0x0B which 
 * in SPI mode controls whether the bus address pins A2, A1, A0 are used.
 * @return 0 on success
 * ****************************************************************************/
uint16_t mcp23017_init_HAEN()
{
	static bool runOnce = false;

	if (!runOnce)
	{
		// Make sure the MCP23S17 devices are set in HAEN mode (individually addressable)
		// Only need to write one device as both are active at the same time to start with.
		mcp23017_write_HAEN(IO_EXP1);
		uint16_t haenSet = mcp23017_read_HAEN(IO_EXP1);
		haenSet = haenSet & mcp23017_read_HAEN(IO_EXP2);
		if(haenSet != 0x01)
		{
			DBG("mcp23017_init_HAEN failed HAEN=%d\r\n", haenSet);
			return -1;
		} else {
			runOnce = true;
		}
	}
	return 0;
}

/**
* Set up the MCP23017 with all CTL line directions set to input (floating) 
* except for RESET & /BUSRQ.
* @param iic The iic file handle
* @note This will leave the /BUSRQ signal asserted (and RESET not asserted.)
*****************************************************************************/
void mcp23017_init(int fd) 
{
#ifdef WTM_PROG_SPI	
	//Set the MCP23S17 devices to HAEN mode (individually addressable)
	mcp23017_init_HAEN();
#endif
    // Set weak 100k pull-ups on the data bus for the case where the 
	// Programmer board is not connected to the Main board
    mcp23017_set_pullup(fd, IO_EXP2, 0xFF, 0x00);
	// preset the addres bus to zero (it doesn't really matter)
	mcp23017_write(fd, IO_EXP1, 0, 0);

	// preset all the control lines to un-asserted except /BUSRQ
	ctl_cache = (~IO_BUSRQ_N)&(~IO_RESET);
	mcp23017_write(fd, IO_EXP2, 0xff, ctl_cache);	// data value = don't care

	// put EX2.A (databus) into input and EX2.B (control) as appropriate
	data_dir_cache = IO_DIR_BUS_AD_IN;
	mcp23017_set_dir(fd, IO_EXP2, data_dir_cache, IO_DIR_BUS_CTL_OFF);	// this will assert /BUSRQ 

	// Cycle the RESET signal here to make sure the FLASH select logic is reset
	mcp23017_write_b(fd, IO_EXP2, ctl_cache|IO_RESET);
	mcp23017_write_b(fd, IO_EXP2, ctl_cache);	

	// Note that the Z80 is likely to glitch the /BUSACK here due
	// to it getting reset, but that is OK.

	// At this point, the Z80 will have yielded control of the bus... 
	// so take over the rest of the control signals.
	// We don't care about /BUSACK because we want this to work even if
	// there is no Z80 on the motherboard.
	mcp23017_set_dir(fd, IO_EXP2, data_dir_cache, IO_DIR_BUS_CTL_ON);	

	// put all bits of EX1 (address bus) into output mode
	mcp23017_set_dir(fd, IO_EXP1, IO_DIR_BUS_AD_OUT, IO_DIR_BUS_AD_OUT);	

}

/**
* De-assert /BUSRQ and cycle the RESET signal.
* @param iic The iic file handle
*****************************************************************************/
void bus_release(int iic)
{
	// Float all the address, data, and control lines EXCEPT for RESET & /BUSRQ
	mcp23017_set_dir(iic, IO_EXP1, IO_DIR_BUS_AD_IN, IO_DIR_BUS_AD_IN);		// release the address lines
	mcp23017_set_dir(iic, IO_EXP2, IO_DIR_BUS_AD_IN, IO_DIR_BUS_CTL_OFF);	// release most of the ctl signals

	// /BUSRQ will have already be asserted 
	ctl_cache = IO_RESET|IO_BUSRQ_N; 		// de-assert /BUSRQ and assert RESET
	mcp23017_write_b(iic, IO_EXP2, ctl_cache);	

	// de-assert the RESET (while keeping /BUSRQ signal de-asserted)
	ctl_cache &= ~IO_RESET;				// RESET=0, leave IO_BUSRQ_N=1
	mcp23017_write_b(iic, IO_EXP2, ctl_cache);
}

/**
* Program a byte in the FLASH
*
* @param iic The iic file handle
* @param addr The address of the byte to be programmed
* @param data The byte value to write into the FLASH
* @note It is assumed that mcp23017_init() is called before this.
*****************************************************************************/
int bus_write_cycle(int iic, uint16_t addr, uint8_t data)
{
	// set the value on the address bus
	mcp23017_write(iic, IO_EXP1, (addr>>8)&0xff, addr&0xff);

	// stage the data to write 
	mcp23017_write_a(iic, IO_EXP2, data);

	if (data_dir_cache!=IO_DIR_BUS_AD_OUT)
	{
		// turn on the data drivers
		data_dir_cache=IO_DIR_BUS_AD_OUT;
		mcp23017_set_dir(iic, IO_EXP2, data_dir_cache, IO_DIR_BUS_CTL_ON);
	}

	// assert /MREQ & /WR
	mcp23017_write_b(iic, IO_EXP2, ctl_cache&(~IO_MREQ_N)&(~IO_WR_N));

	// un-assert /MREQ & /WR
	mcp23017_write_b(iic, IO_EXP2, ctl_cache);

	return 0;
}

/**
* Read a byte from flash at address: addr.
* @param iic The iic file handle
* @param addr The address of the byte to be read
* @return The byte value read from the FLASH
* @note It is assumed that mcp23017_init() is called before this.
*****************************************************************************/
uint8_t bus_read_cycle(int iic, uint16_t addr)
{
	// set the value on the address bus
	mcp23017_write(iic, IO_EXP1, (addr>>8)&0xff, addr&0xff);

	if (data_dir_cache!=IO_DIR_BUS_AD_IN)
	{
		// turn off the data drivers
		data_dir_cache=IO_DIR_BUS_AD_IN;
		mcp23017_set_dir(iic, IO_EXP2, data_dir_cache, IO_DIR_BUS_CTL_ON);
	}

	// assert /MREQ & /RD
	mcp23017_write_b(iic, IO_EXP2, ctl_cache&(~IO_MREQ_N)&(~IO_RD_N));

	// READ the data bus
	int rc = mcp23017_read(iic, IO_EXP2);

	// un-assert /MREQ & /RD
	mcp23017_write_b(iic, IO_EXP2, ctl_cache);

	return rc&0xff;
}



//***************************************************************************
//***************************************************************************
//***************************************************************************
//***************************************************************************




/**
* Send the security sequence to the FLASH
* @param iic The iic file handle
* @param cmd The value of the command byte to send to the FLASH
*****************************************************************************/
int flash_send_sdp(int iic, uint8_t cmd)
{
	int rc = 0;
	rc |= bus_write_cycle(iic, 0x5555, 0xAA);
	rc |= bus_write_cycle(iic, 0x2AAA, 0x55);
	rc |= bus_write_cycle(iic, 0x5555, cmd);
	return rc;
}

/**
* Read and print the FLASH Product ID.
* @param iic The iic file handle
* @return The manufacturer (in the MSB) and the product (in the LSB) 
*	of the FLASH
*****************************************************************************/
uint16_t flash_read_product_id(int iic)
{
	flash_send_sdp(iic, 0x90);				// Software ID read

	uint16_t mfg = bus_read_cycle(iic, 0);	// read a byte from address 0
	uint16_t prod = bus_read_cycle(iic, 1);	// read a byte from address 1

	flash_send_sdp(iic, 0xf0);				// Software ID Exit

	return (mfg<<8)|prod;
}

/**
* Erase the entire FLASH chip.
* @param iic The iic file handle
* @note This operation can take up to 100msec.
*****************************************************************************/
int flash_chip_erase(int iic)
{
	flash_send_sdp(iic, 0x80);
	flash_send_sdp(iic, 0x10);
	delayMicroseconds(200000); // some extra margin!
	return 0;
}

/**
* Program one single byte data into address addr.
* @param iic The iic file handle
* @param addr The address of the byte to be programmed
* @param data The byte value to write into the FLASH
* @note This operation can take up to 20usec.
*****************************************************************************/
int flash_program_byte(int iic, uint16_t addr, uint8_t data)
{
#if 1
	if (data == 0xff)
		return 0;		// no need to program 0xff since that is the erased state
#endif

	flash_send_sdp(iic, 0xa0);
	bus_write_cycle(iic, addr, data);
	delayMicroseconds(21);		// some extra margin!
	return 0;
}

/**
* Modus operandi:
*
* Open the I2C port.
* Initialize the MCP23017s and assert /BUSRQ to steal the Z80 BUS.
* Read and print the FLASH product ID.
* If the ID is invalid, exit with an error.
* Bulk-erase the entire FLASH chip.
* Read and program one byte at-a-time from a binary STDIN stream.
*	The first byte will be written into address 0, the second to address 1,...
* When hit EOF read the entire contents of the FLASH back in, compare it 
*	to the original data, and print a success/fail message.
* Release the Z80 bus.
*****************************************************************************/
#define BUF_SIZE (16384)
// #define BUF_SIZE 65536; // Not enough memory on ESP2866 for this.
int goProgrammer()
{
    int     iic = 0;
	
	int dbgCount = 1;
	DBG("wait %d\r\n", dbgCount++);
    delay(1000);
	char	buf[BUF_SIZE];

	DBG("wait %d\r\n", dbgCount++);
    delay(1000);
    // iic = openIIC(I2C_DEV);
	mcp23017_init(iic);

	DBG("wait %d\r\n", dbgCount++);
    delay(1000);
	uint16_t d = flash_read_product_id(iic);
	if (d != 0xbfb5 && d != 0xbfb6 && d != 0xbfb7)
	{
		printf("Invalid FLASH signature: 0x%04x != 0xbfb5\r\n", d);
		exit(1);
	}
	DBG("wait %d\r\n", dbgCount++);
    delay(1000);

// Read BLOCK_SIZE Bytes from file
#if 0
bus_release(iic);
exit(0);
#endif

	uint16_t addr = 0;
#if 0

#if 1
	printf("Erasing entire chip\r\n");
	flash_chip_erase(iic);
#endif

#if 1

	printf("Programming\r\n");
	// Program the flash direct from stdin 
	uint8_t b;
	while(read(fileno(stdin), &b, 1) == 1)
	{
		if (addr%16 == 0)
			printf("%s%04x:", addr==0?"":"\r\n", addr);
		printf(" %02x", b);

		buf[addr] = b;
		flash_program_byte(iic, addr, b);
		++addr;

		if (addr > sizeof(buf))
		{
			printf("program file is too big, limiting to %d bytes\r\n", sizeof(buf));
			break;
		}
	}
	printf("\r\n");

	printf("**********************************************************\r\n");
#endif

#endif

	printf("Reading\r\n");
	char	rbuf[BUF_SIZE];
	DBG("wait %d\r\n", dbgCount++);
    delay(1000);


	// read the flash back again
	if (addr == 0)
		addr = 0x100;	// just read 0x100 bytes if we didn't write anything

	uint16_t raddr = 0;
	while(raddr < addr)
	{
		rbuf[raddr] = bus_read_cycle(iic, raddr);

		if (raddr%16 == 0)
			printf("%s%04x:", raddr==0?"":"\r\n", raddr);
		printf(" %02x", rbuf[raddr]);

		++raddr;
	}
	// printf("\r\n");
	DBG("wait %d\r\n", dbgCount++);
    delay(1000);


	bus_release(iic);

	DBG("wait %d\r\n", dbgCount++);
    delay(1000);

	printf("Verifying.\r\n");
	if (memcmp(buf, rbuf, addr) == 0)
		printf("SUCCESS!\r\n");
	else
		printf("FAILED!\r\n");

	return 0;
}
