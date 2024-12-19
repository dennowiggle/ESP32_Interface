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

#ifndef __Z80_IO_BUS_H__
#define __Z80_IO_BUS_H__

#include <stdint.h>

void z80DataPortDirSetup(uint8_t portSetup);
uint8_t z80DataPortRead();
void z80DataPortwrite(uint8_t data);
void z80IorqSetup();
void handleZ80Iorq();
void handleZ80IorqTest();






#endif /* __Z80_IO_BUS_H__ */