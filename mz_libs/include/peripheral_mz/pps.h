/*
		FileName: pps.h
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

#ifndef PPS_H
#define	PPS_H

#include <xc.h>

/*
Example Code:

   // Assign UART 2 signals onto pins using PPS
    PPSInput(2,U2RX, RPA9);  //Assign U2RX to pin RPA09
    PPSInput(3,U2CTS,RPC3);  //Assign U2CTS to pin RPC3
    PPSOutput(4,RPC4,U2TX);   //Assign U2TX to pin RPC4
    PPSOutput(1,RPB15,U2RTS); //Assign U2RTS to pin RPB15

 */

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* PPS_H */

