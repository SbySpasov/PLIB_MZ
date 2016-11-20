/*
		FileName: plib_mz.h
*/
/*
    plib_mz for PIC32MZ microcontrollers - Copyright (C) 2016
                                            Spas Spasov.

    This file is part of plib_mz.

    "plib_mz" is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    "plib_mz" is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
   Concepts and parts of this file have been contributed by Spas Spasov.
 */

#ifndef _PERIPHERAL_LIBRARY_MZ_MASTER_HEADER_FILE
#define _PERIPHERAL_LIBRARY_MZ_MASTER_HEADER_FILE

#include <xc.h>

#ifndef __C32__
#define __C32__ 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__PIC32MZ__)
#include "peripheral_mz/adc12.h"
#include "peripheral_mz/cmp.h"
#include "peripheral_mz/cvref.h"
#include "peripheral_mz/dma.h"
#include "peripheral_mz/i2c.h"
#include "peripheral_mz/incap.h"
#include "peripheral_mz/int.h"
#include "peripheral_mz/lock.h"
#include "peripheral_mz/nvm.h"
#include "peripheral_mz/osc.h"
#include "peripheral_mz/outcompare.h"
#include "peripheral_mz/pcache.h"
#include "peripheral_mz/pmp.h"
#include "peripheral_mz/ports.h"
#include "peripheral_mz/pps.h"
#include "peripheral_mz/power.h"
#include "peripheral_mz/reset.h"
//#include "peripheral/rtcc.h"
#include "peripheral_mz/spi.h"
#include "peripheral_mz/system.h"
#include "peripheral_mz/timer.h"
#include "peripheral_mz/uart.h"
#include "peripheral_mz/wdt.h"
//#include "peripheral/eth.h"
//#include "peripheral/can.h"
//#include "peripheral/sqi.h"
#else
#error "Device not supported by the plib_mz peripheral library"
#endif
#ifdef __cplusplus
}
#endif

#endif
