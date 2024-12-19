//****************************************************************************
// 
//    Updated DBG debugging macro by Denno Wiggle aka WTM
// 
//****************************************************************************
//
//    Original DBG debugging macro
//
//    Copyright (C) 2011  John Winans
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

#ifndef _DBG_H_
#define _DBG_H_

#include "wtmSerialx.h"
#include "defines.h"

#define ENABLE_DBG

#ifdef WTM_ENABLE_DBG
    extern wtmSerialX* console;
    #define DBGC(...) do { console->printf("DBG: %s %s() %d: ", __FILE__, __FUNCTION__, __LINE__); logger->printf(__VA_ARGS__); logger->flush(); } while(0)
    extern wtmSerialX* logger;
    #define DBG(...) do { logger->printf("DBG: %s %s() %d: ", __FILE__, __FUNCTION__, __LINE__); logger->printf(__VA_ARGS__); logger->flush(); } while(0)
    #else
        #ifdef ENABLE_DBG
        #define DBG(...) do { printf("*** DBG: %s %s() %d: ", __FILE__, __FUNCTION__, __LINE__); printf(__VA_ARGS__); fflush(stdout); } while(0)
        #else
        #define DBG(...) do { } while(0)
        #endif
    #endif
#endif /* _DBG_H_ */

