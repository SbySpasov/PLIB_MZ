/*
		FileName: rtcc.h
*/
/*
    plib_mz for PIC32MZ microcontrollers - Copyright (C) 2016-2019
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

#ifndef RTCC_H
#define	RTCC_H

#include <xc.h>
#include "../peripheral_mz/int.h"

#ifdef	__cplusplus
extern "C" {
#endif

// RTCC definitions

// union/structure for read/write of time into the RTCC device
typedef union
{
    struct
    {
        unsigned char   rsvd;       // reserved for future use. should be 0
        unsigned char   sec;        // BCD codification for seconds, 00-59
        unsigned char   min;        // BCD codification for minutes, 00-59
        unsigned char   hour;       // BCD codification for hours, 00-24
    };                              // field access
    unsigned char       b[4];       // byte access
    unsigned short      w[2];       // 16 bits access
    unsigned long       l;          // 32 bits access
}rtccTime;

// union/structure for read/write of date into the RTCC device
typedef union
{
    struct
    {
        unsigned char   wday;       // BCD codification for day of the week, 00-06
        unsigned char   mday;       // BCD codification for day of the month, 01-31
        unsigned char   mon;        // BCD codification for month, 01-12
        unsigned char   year;       // BCD codification for years, 00-99
    };                              // field access
    unsigned char       b[4];       // byte access
    unsigned short      w[2];       // 16 bits access
    unsigned long       l;          // 32 bits access
}rtccDate;

// valid values of alarm repetition for the RTCC device
typedef enum
{
    RTCC_RPT_HALF_SEC,      // repeat alarm every half second
    RTCC_RPT_SEC,           // repeat alarm every second
    RTCC_RPT_TEN_SEC,       // repeat alarm every ten seconds
    RTCC_RPT_MIN,           // repeat alarm every minute
    RTCC_RPT_TEN_MIN,       // repeat alarm every ten minutes
    RTCC_RPT_HOUR,          // repeat alarm every hour
    RTCC_RPT_DAY,           // repeat alarm every day
    RTCC_RPT_WEEK,          // repeat alarm every week
    RTCC_RPT_MON,           // repeat alarm every month
    RTCC_RPT_YEAR           // repeat alarm every year (except when configured for Feb 29th.)
}rtccRepeat;

// results returned by initialization functions
typedef enum
{
    RTCC_CLK_ON,            // success, clock is running
    RTCC_SOSC_NRDY,         // SOSC not running
    RTCC_CLK_NRDY,          // RTCC clock not running
    RTCC_WR_DSBL,           // WR is disabled
}rtccRes;

// RTCC Output Data Select pin
typedef enum
{
    RTCC_LPRC,              // RTCC uses the internal 32 kHz oscillator (LPRC)
    RTCC_SOSC,              // RTCC uses the external 32.768 kHz Secondary Oscillator (SOSC)
    RTCC_RESERVED1,         // Reserved
    RTCC_RESERVED2,         // Reserved
}rtccClockSel;

// RTCC Output Data Select pin
typedef enum
{
    RTCC_ALARM_PULSE,       // Alarm Pulse is presented on the RTCC pin when the alarm interrupt is triggered
    RTCC_SECONDS_CLOCK,     // Seconds Clock is presented on the RTCC pin
    RTCC_RTCC_CLOCK,        // RTCC Clock is presented on the RTCC pin
    RTCC_RESERVED,          // Reserved
}rtccOutSel;

// RTCC interface functions

// low level control functions

/*********************************************************************
 * Function:        int RtccGetAlarmPulse(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          1 if the Alarm Pulse output is asserted
 *                  0 otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current state of the output Alarm Pulse.
 *
 * Note:            The alarm has to be enabled for this function to return the current state of the Alarm Pulse output.
 *
 * Example:         int alrmPulse=RtccGetAlarmPulse();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetAlarmPulse(void)
{

    return RTCALRMbits.PIV!=0;
}

/*********************************************************************
 * Function:        void RtccOutputEnable(int enable)
 *
 * PreCondition:    None
 *
 * Input:           enable -  boolean to enable/disable the RTCC output pin
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function enables/disables the Output pin of the RTCC.
 *
 * Note:            The RTCC has to be enabled for the output to actually be active.
 *
 * Example:         RtccOutputEnable(1);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccOutputEnable(int enable)
{

    if( enable )
    {
        RTCCONSET=_RTCCON_RTCOE_MASK;
    }
    else
    {
        RTCCONCLR=_RTCCON_RTCOE_MASK;
    }
}

/*********************************************************************
 * Function:        int RtccGetOutputEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE if Output is enabled, FALSE otherwise.
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the enabled/disabled status of the RTCC Output pin.
 *
 * Note:            None
 *
 * Example:         int isOutEnabled=RtccGetOutputEnable();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetOutputEnable(void)
{

    return  RTCCONbits.RTCOE!=0;
}

/*********************************************************************
 * Function:        int RtccGetWrEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE if the RTCC is unlocked, FALSE if locked
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current status of the RTCC write enable bit.
 *
 * Note:            None
 *
 * Example:         int isWrEnabled=RtccGetWrEnable();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetWrEnable(void)
{

    return  RTCCONbits.RTCWREN!=0;
}

/*********************************************************************
 * Function:        int RtccGetSync(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE if the SYNC signal is asserted, FALSE otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current status of the RTCCON SYNC bit.
 *
 * Note:            None
 *
 * Example:         int isSync=RtccGetSync();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetSync(void)
{

    return  RTCCONbits.RTCSYNC!=0;
}


/*********************************************************************
 * Function:        int RtccGetHalfSecond(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE if the RTCC is in the second HALF SECOND  interval, FALSE otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current status of the RTCCON HALFSEC bit.
 *
 * Note:            None
 *
 * Example:         int is2HalfSec=RtccGetHalfSecond();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetHalfSecond(void)
{

    return  RTCCONbits.HALFSEC!=0;
}

/*********************************************************************
 * Function:        int RtccGetAlrmSync(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE if the AlrmSync signal is asserted, FALSE otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current status of the RTCALRM ALRMSYNC bit.
 *
 * Note:            None
 *
 * Example:         int isSync=RtccGetAlrmSync();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetAlrmSync(void)
{

    return  RTCALRMbits.ALRMSYNC!=0;
}

/*********************************************************************
 * Function:        void RtccClockSelect(rtccClockSel clockSel)
 *
 * PreCondition:    None
 *
 * Input:           clockSel - enum to select the oscillator clock for RTCC
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function selects the oscillator clock or the RTCC input.
 *
 * Note:            
 *
 * Example:         RtccClockSelect(RTCC_SOSC);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccClockSelect(rtccClockSel clockSel)
{
unsigned int tmp_clock = 0;

    if( clockSel == RTCC_LPRC    ||
        clockSel == RTCC_SOSC )
    {
        RTCCONCLR = _RTCCON_RTCCLKSEL_MASK;
        tmp_clock = (unsigned int)clockSel << _RTCCON_RTCCLKSEL_POSITION;
        RTCCONSET = tmp_clock;
    }
}

/*********************************************************************
 * Function:        void RtccSelectPulseOutput(rtccOutSel secPulse)
 *
 * PreCondition:    None
 *
 * Input:           secPulse - enum to select the seconds/alarm pulse as output
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function selects the seconds clock pulse or the alarm pulse as the function of the RTCC output pin.
 *
 * Note:            The RTCC has to be enabled for the output to actually be active.
 *
 * Example:         RtccSelectPulseOutput(RTCC_ALARM_PULSE);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSelectPulseOutput(rtccOutSel secPulse)
{
unsigned int tmp_pulse = 0;

    if( secPulse == RTCC_ALARM_PULSE    ||
        secPulse == RTCC_SECONDS_CLOCK  ||
        secPulse == RTCC_RTCC_CLOCK )
    {
        RTCCONCLR = _RTCCON_RTCOUTSEL_MASK;
        tmp_pulse = (unsigned int)secPulse << _RTCCON_RTCOUTSEL_POSITION;
        RTCCONSET = tmp_pulse;
    }
}

/*********************************************************************
 * Function:        int RtccGetCalibration(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Current value of the RTCC calibration field.
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the value that the RTCC uses in the
 *                  auto-adjust feature, once every minute.
 *                  The calibration value is a signed 10 bits value, [-512, +511].
 *
 * Note:            None
 *
 * Example:         int currCal=RtccGetCalibration();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetCalibration(void)
{

    return  RTCCONbits.CAL;
}

/*********************************************************************
 * Function:        int RtccGetEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          the current enabled/disabled status of the RTCC module.
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the RTCCON.ON bit anded with RTCCLKON.
 *
 * Note:            None
 *
 * Example:         int isEnabled=RtccGetEnable();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetEnable(void)
{

    return(RTCCON&(_RTCCON_ON_MASK|_RTCCON_RTCCLKON_MASK));
}

/*********************************************************************
 * Function:        rtccRes RtccGetClkStat(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          RTCC_CLK_ON if the RTCC clock is running (the oscillator output is presented to the RTCC module).
 *                  an clock status otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the RTCCON.ON bit anded with RTCCLKON.
 *
 * Note:            None
 *
 * Example:         int isClkOn=RtccGetClkStat();
 ********************************************************************/
extern inline rtccRes __attribute__((always_inline)) 
RtccGetClkStat(void)
{

    if( OSCCONbits.SOSCEN )
    {
        if( !(CLKSTATbits.SOSCRDY) )
        {
            return RTCC_SOSC_NRDY;
        }
    }
    else if( !(CLKSTATbits.LPRCRDY) )
    {
        return RTCC_SOSC_NRDY;
    }
    
    if( !(RTCCONbits.RTCCLKON) )
    {
        return RTCC_CLK_NRDY;
    }

    return RTCC_CLK_ON;
}

/*********************************************************************
 * Function:        void RtccWrEnable(int enable)
 *
 * PreCondition:    None
 *
 * Input:           enable - boolean to enable/disable the RTCC updates.
 *
 * Output:          None.
 *
 * Side Effects:    None
 *
 * Overview:        The function enables the updates to the RTCC
 *                  time registers and ON control bit.
 *
 * Note:            - The write can be enabled by performing a specific unlock sequence.
 *                  In order to succeed, this sequence need not be interrupted by other memory accesses
 *                  (DMA transfers, interrupts, etc).
 *                  - Interrupts and DMA transfers that might disrupt the write unlock sequence are disabled
 *                  shortly for properly unlocking the device.
 *
 * Example:        RtccWrEnable(1);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccWrEnable(int enable)
{

    if( enable )
    {
        mSysUnlockOpLock(RTCCONSET=_RTCCON_RTCWREN_MASK);
    }
    else
    {
        RTCCONCLR = _RTCCON_RTCWREN_MASK;
    }
}

// Helpers
/*********************************************************************
 * Function:        rtccRes _RtccWaitClockOff(void)
 *
 * PreCondition:    None
 * Input:           None
 * Output:          actual clock status
 * Side Effects:    None
 * Overview:        Helper to wait until RTCCON.RTCCLKON signal is cleared.
 * Note:            The function relies on the OSCCON.SOSCRDY bit functionality
 ********************************************************************/
extern inline rtccRes __attribute__((always_inline)) 
_RtccWaitClockOff(void)
{
unsigned int t0, t1;
unsigned int RtccWaitClkSteps = 1080;
unsigned int RtccWaitClkStepSz = 7;

	// if the SOSC is off, we cannot use the SOSCRDY bit and there's no need to wait for RTCCCLKON
	if( OSCCONbits.SOSCEN )
	{	
		while( RtccWaitClkSteps-- )
		{
			if( !(CLKSTATbits.SOSCRDY) || !(RTCCONbits.RTCCLKON) )
			{ // if SOSC is just warming up, nothing much we can do; same if RTCC clock is stopped, we're done
				break;	// nothing much we can do
			}

			// at this point we don't really know if the RTCCLK didn't stop yet
			// or the SOSC clock just vanished. Note that SOSCRDY doesn't clear by itself
			// .... we have to wait
			t0 = ReadCoreTimer();
			do
			{
				t1=ReadCoreTimer();
			}while( (t1-t0) < RtccWaitClkStepSz );
		}
	}

	return RtccGetClkStat();
}

/*********************************************************************
 * Function:        __inline__ void _RtccWaitSync(void)
 *
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Inline helper to wait until RTCCON.RTCSYNC signals read/write is safe.
 * Note:            In order to be sure that the write/read op is safe, interrupts should be disabled
 * 					or kept very short (worst case scenario, sync can be asserted for 32 RTCC clocks,
 * 					i.e. almost 1ms, so it's not advisable to disable the interrupts for such a long period
 * 					of time. Care must be taken under these circumstances).
 ********************************************************************/
extern inline void __attribute__((always_inline))  
_RtccWaitSync(void)
{

	while(RtccGetSync());
}

/*********************************************************************
 * Function:        __inline__ void _RtccWaitAlrmSync(void)
 *
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Inline helper to wait until RTCALRM.ALRMSYNC signals read/write is safe.
 * Note:            In order to be sure that the write/read op is safe, interrupts should be disabled
 * 					or kept very short (worst case scenario, sync can be asserted for 32 RTCC clocks,
 * 					i.e. almost 1ms, so it's not advisable to disable the interrupts for such a long period
 * 					of time. Care must be taken under these circumstances).
 ********************************************************************/
extern inline void __attribute__((always_inline))  
_RtccWaitAlrmSync(void)
{

	while(RtccGetAlrmSync());
}

/*********************************************************************
 * Function:        __inline__ void _RtccStop(void)
 *
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Inline helper to stop the module. Waits for the clock to be actually stopped.
 * Note:            The function relies on the OSCCON.SOSCRDY bit functionality
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
_RtccStop(void)
{

	RTCCONCLR=_RTCCON_ON_MASK;	// stop the module
	_RtccWaitClockOff();
}

/*********************************************************************
 * Function:        void _RtccWriteInAlarmWindow(volatile unsigned int* pR1, unsigned int v1, volatile unsigned int* pR2, unsigned int v2)
 *
 * PreCondition:    None
 *
 * Input:           pR1		- if non NULL, address of a register to update when it's safe to do so
 * 					v1		- value to be written at pR1
 * 					pR2		- if non NULL, address of a register to update when it's safe to do so
 * 					v2		- value to be written at pR2
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Helper to perform a write in a safe alarm window.
 *
 * Note:            Interrupts are disabled shortly when properly probing the ALRMSYNC is needed.
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
_RtccWriteInAlarmWindow(volatile unsigned int* pR1, unsigned int v1, volatile unsigned int* pR2, unsigned int v2)
{
int	intLev=0;
int	restoreInts=0;

	if( RTCCONbits.ON )
	{	// device is enabled, there's incoming clock, have to synchronize access
		restoreInts=1;

		while(1)
		{
			while( RTCALRMbits.ALRMSYNC );	// wait sync go away
			intLev=INTDisableInterrupts();
			if( !(RTCALRMbits.ALRMSYNC) )
			{
				break;		// we're in the safe window
			}

			INTRestoreInterrupts(intLev);	// restore ints and continue waiting
		}
	}
	else
	{
		_RtccWaitClockOff();		// make sure the clock is not running
	}

	// safe to update values

	if( pR1 )
	{
		*pR1=v1;
	}
	if( pR2 )
	{
		*pR2=v2;
	}
	if( restoreInts )
	{
		INTRestoreInterrupts(intLev);
	}
}

/*********************************************************************
 * Function:        void _RtccSetAlrmTimeAndDate(const rtccTime* pTm, const rtccDate* pDt)
 *
 * PreCondition:    rtccTime and rtccDate structure fields have to have proper values:
 * 						- sec:	BCD codification, 00-59
 * 						- min:  BCD codification, 00-59
 * 						- hour: BCD codification, 00-24
 * 						- wday:	BCD codification, 00-06
 * 						- mday: BCD codification, 01-31
 * 						- mon: BCD codification, 01-12
 * Input:           pTm - pointer to a constant rtccTime union
 * 					pTd	- pointer to a constant rtccDate union
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        The function sets the current time and date in the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 * 					to safely perform the update of the ALRMTIME register.
 * 					- Interrupts are disabled shortly when properly probing the RTCSYNC/ALRMSYNC needed.
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
_RtccSetAlrmTimeAndDate(const rtccTime* pTm, const rtccDate* pDt)
{
unsigned int		  v1, v2;
volatile unsigned int *pR1, *pR2;

	if( pTm )
	{
		v1=pTm->l;
		pR1=&ALRMTIME;
	}
	else
	{
		v1=0;
		pR1=0;
	}
	
	if( pDt )
	{
		v2=pDt->l;
		pR2=&ALRMDATE;
	}
	else
	{
		v2=0;
		pR2=0;
	}

	_RtccWriteInAlarmWindow(pR1, v1, pR2, v2);
}

/*********************************************************************
 * Function:        void _RtccSetTimeAndDate(const rtccTime* pTm, const rtccDate* pDt)
 *
 * PreCondition:    rtccTime and rtccDate structures fields have to have proper values:
 * 						- sec:	BCD codification, 00-59
 * 						- min:  BCD codification, 00-59
 * 						- hour: BCD codification, 00-24
 * 						- wday:	BCD codification, 00-06
 * 						- mday: BCD codification, 01-31
 * 						- mon: BCD codification, 01-12
 * 						- year: BCD codification, 00-99
 * Input:           pTm - pointer to a constant rtccTime union
 * 					pDt	- pointer to a constant rtccDate union
 *
 * Output:          None
 * Side Effects:    None
 * Overview:        The function sets the current time and date in the RTCC device.
 *
 * Note:            - The write is successful only if Wr Enable is set.
 * 					The function will enable the write itself, if needed.
 * 					- The device could be stopped in order
 * 					to safely perform the update of the RTC time register.
 * 					However, the device status will be restored but	the routine won't wait
 * 					for the CLK to be running before returning. User has to check RtccGetClkOn() (will take approx 30us).
 * 					- The routine disables the interrupts for a very short time to be able
 * 					to update the time and date registers.
 ********************************************************************/
extern inline void __attribute__((always_inline))
_RtccSetTimeAndDate(const rtccTime* pTm, const rtccDate* pDt)
{
int	wasWrEn;
int	intLevel;

	if( !(wasWrEn=RtccGetWrEnable()) )
	{
		RtccWrEnable(1);			// have to allow the WRTEN in order to write the new value
	}

	if( RTCCONbits.ON )
	{	// device is ON
		intLevel=INTDisableInterrupts();

		// Poll RTCSYNC.
		if( RTCCONbits.RTCSYNC || (RTCALRMbits.ALRMEN && RTCALRMbits.AMASK==RTCC_RPT_HALF_SEC) )
		{	// a rollover event taking place, we cannot perform the write now
			INTRestoreInterrupts(intLevel);				// restore the interrupts, we're going to wait for a while
			_RtccStop();	// turn module off before updating the time
						// make sure RTCC clock is stopped
			if( pTm )
			{
				RTCTIME=pTm->l;		// update the RTCTIME value
			}
			if( pDt )
			{
				RTCDATE=pDt->l;		// update the RTCDATE value
			}
			RTCCONSET = _RTCCON_ON_MASK;	// don't wait for the clock to be back on
		}
		else
		{	// there's no rollover now, safe to write
			if( pTm )
			{
				RTCTIME = pTm->l;		// update the RTCTIME value
			}
			if(pDt)
			{
				RTCDATE = pDt->l;		// update the RTCDATE value
			}
			INTRestoreInterrupts(intLevel);
		}
	}
	else
	{	// device is OFF, so we don't have to disable the interrupts
		_RtccWaitClockOff();	// make sure the clock is not running
		if( pTm )
		{
			RTCTIME = pTm->l;		// update the RTCTIME value
		}
		if( pDt )
		{
			RTCDATE = pDt->l;		// update the RTCDATE value
		}
	}

	// now restore the status
	if( !wasWrEn )
	{
		RtccWrEnable(0);	// disable
	}
}
// end of Helpers

/*********************************************************************
 * Function:        rtccRes RtccEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          RTCC_CLK_ON if the RTCC was enabled and the RTCC clock is running,
 *                  a clock status code otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function enables the RTCC.
 *
 * Note:            - The write operations have to be enabled in order to be able to toggle the ON control bit.
 *                  Otherwise the function will fail.
 *                  See RtccWrEnable() function.
 *                  - The function doesn't wait for the RTC clock to be on.
 *
 * Example:         rtccRes clkStat=RtccEnable();
 ********************************************************************/
extern inline rtccRes __attribute__((always_inline)) 
RtccEnable(void)
{
rtccRes	res;

	if( RTCCONbits.ON )
	{
		res = RtccGetClkStat();
	}
	else if( RtccGetWrEnable() )
	{
		RTCCONSET = _RTCCON_ON_MASK;
		res = RtccGetClkStat();
	}
	else
	{
		res = RTCC_WR_DSBL;
	}

	return res;
}

/*********************************************************************
 * Function:        int RtccDisable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE if the RTCC was disabled,
 *                  FALSE otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function disables the RTCC.
 *
 * Note:            1. The write operations have to be enabled in order to be able to toggle the ON control bit.
 *                  Otherwise the function will fail.
 *                  See RtccWrEnable() function.
 *                  2. When ON control bit is set to 0, RTCCON.RTCSYNC, RTCCON.HALFSEC and
 *                  RTCCON.RTCOE are asynchronously reset.
 *                  - The function waits for the RTC clock to be off.
 *
 * Example:         RtccDisable();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccDisable(void)
{
int	res;

	if( RTCCONbits.ON )
	{
		if( RtccGetWrEnable() )
		{
			_RtccStop();
			res=1;
		}
		else
		{
			res=0;
		}
	}
	else
	{
		_RtccWaitClockOff();	// make sure clock is stopped
		res=1;
	}

	return res;
}

/*********************************************************************
 * Function:        void RtccSetCalibration(int drift)
 *
 * PreCondition:    drift has to fit into signed 10 bits representation
 *
 * Input:           drift   - value to be added/subtracted to perform calibration
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function updates the value that the RTCC uses in the
 *                  auto-adjust feature, once every minute.
 *                  The drift value acts as a signed value, [-512, +511], 0 not having
 *                  any effect.
 *
 * Note:            - Writes to the RTCCON.CAL[9:0] register should  only occur when the timer
 *                  is turned off or immediately or after the edge of the seconds pulse
 *                  (except when SECONDS=00 - due to the possibility of the auto-adjust event).
 *                  In order to speed-up the process, the API function performs the reading
 *                  of the HALFSEC field.
 *                  - The function may block for half a second, worst case, when called
 *                  at the start of the minute.
 *                  - A write to the SECONDS value resets the state of the calibration and the prescaler.
 *                  If calibration just occurred, it will occur again at the prescaler rollover.
 *                  - Interrupts can not be disabled for such a long period. However, long interrupt routines
 *                  can interfere with the proper functioning of the device. Care must be taken.
 *
 * Example:         RtccSetCalibration(10);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetCalibration(int drift)
{
unsigned int ulDrift=drift<<_RTCCON_CAL_POSITION;

	if( RTCCONbits.ON )
	{	// device is running
		unsigned int	t0, t1;
		do
		{
			t0=RTCTIME;
			t1=RTCTIME;
		}while(t0!=t1);		// read valid time value

		if( (t0&(_RTCTIME_SEC01_MASK|_RTCTIME_SEC10_MASK))==00 )
		{	// we're at second 00, wait auto-adjust to be performed
			while(!RtccGetHalfSecond());	// wait until second half...
		}
	}
	else
	{	// device is off
		_RtccWaitClockOff();	// make sure the clock is stopped
	}

	// update the CAL value
	RTCCONCLR=_RTCCON_CAL_MASK;
	RTCCONSET=ulDrift;
}

/*********************************************************************
 * Function:        void RtccAlarmPulseHigh(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the initial value of the output Alarm Pulse to logic 1.
 *
 * Note:            1. The RTCC has to be enabled for the output to actually be active.
 *                  2. This Alarm Pulse output is writable only when the alarm is disabled.
 *                  3. The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  4. Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccAlarmPulseHigh();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccAlarmPulseHigh(void)
{

	_RtccWriteInAlarmWindow(&RTCALRMSET, _RTCALRM_PIV_MASK, 0, 0);
}

/*********************************************************************
 * Function:        void RtccAlarmPulseLow(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the initial value of the output Alarm Pulse to logic 0.
 *
 * Note:            1. The RTCC has to be enabled for the output to actually be active.
 *                  2. This Alarm Pulse output is writable only when the alarm is disabled.
*                   3. The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  4. Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccAlarmPulseLow();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccAlarmPulseLow(void)
{

	_RtccWriteInAlarmWindow(&RTCALRMCLR, _RTCALRM_PIV_MASK, 0, 0);
}

/*********************************************************************
 * Function:        void RtccAlarmPulseToggle(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function toggles the value of the output Alarm Pulse.
 *
 * Note:            1. The RTCC has to be enabled for the output to actually be active.
 *                  2. This Alarm Pulse output is writable only when the alarm is disabled.
*                   3. The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  4. Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccAlarmPulseToggle();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccAlarmPulseToggle(void)
{

	_RtccWriteInAlarmWindow(&RTCALRMINV, _RTCALRM_PIV_MASK, 0, 0);
}


// high level control functions

/*********************************************************************
 * Function:        rtccRes RtccInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          RTCC_CLK_ON if the RTCC clock is actually running
 *                  a clock status code otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function initializes the RTCC device. It starts the RTCC clock,
 *                  enables the RTCC and disables RTCC write. Disables the Alarm and the OE.
 *                  Clears the alarm interrupt flag and disables the alarm interrupt.
 *
 * Note:            It usually takes 4x256 clock cycles (approx 31.2 ms) for the oscillator signal to be available
 *                  to the RTCC. The user must make sure that the clock is actually running using RtccGetClkStat()
 *                  before expecting the RTCC to count.
 *
 * Example:         rtccRes res=RtccInit();
 ********************************************************************/
extern inline rtccRes __attribute__((always_inline)) 
RtccInit(rtccClockSel clockSel)
{

    if( (clockSel == RTCC_LPRC) || (clockSel == RTCC_SOSC) )
    {
        INTEnable(INT_SOURCE_RTCC, INT_DISABLED);	// disable RTCC interrupts
        // be sure the SOSC is enabled, enable RTCC writes
        if( clockSel == RTCC_SOSC )
        {
            mSysUnlockOpLock((OSCCONSET=_OSCCON_SOSCEN_MASK, RTCCONSET=_RTCCON_RTCWREN_MASK));
        }
        else
        {
            mSysUnlockOpLock((RTCCONSET=_RTCCON_RTCWREN_MASK));
        }
        _RtccStop();
        RTCALRMCLR=_RTCALRM_ALRMEN_MASK;// disable the alarm
        INTClearFlag(INT_SOURCE_RTCC);		// clear the interrupt flag
        RTCCONSET=_RTCCON_ON_MASK;
        RtccClockSelect(clockSel);
        RtccWrEnable(0);
        RtccOutputEnable(0);
    }
	return RtccGetClkStat();
}

/*********************************************************************
 * Function:        rtccRes RtccOpen((unsigned long tm, unsigned long dt, int drift)
 *
 * PreCondition:    tm an unsigned long containing the fields of a valid rtccTime structure:
 *                      - sec:  BCD codification, 00-59
 *                      - min:  BCD codification, 00-59
 *                      - hour: BCD codification, 00-24
 *          dt is an unsigned long conatining the valid fields of a rtccDate structure:
 *                      - wday: BCD codification, 00-06
 *                      - mday: BCD codification, 01-31
 *                      - mon: BCD codification, 01-12
 *                      - year: BCD codification, 00-99
 *          drift has to fit into signed 10 bits representation
 *
 * Input:               tm - the time value to be set
 *          dt - the date value to be set
 *          drift   - value to be added/subtracted to perform calibration
 *
 * Output:          RTCC_CLK_ON if the RTCC clock is actually running
 *                  a clock status code otherwise
 *
 * Side Effects:    None
 *
 * Overview:        The function initializes the RTCC device. It starts the RTCC clock, sets the desired time and calibration
 *                  and enables the RTCC. Disables the Alarm and the OE and further RTCC writes.
 *                  Clears the alarm interrupt flag and disables the alarm interrupt.
 *
 * Note:            It usually takes 4x256 clock cycles (approx 31.2 ms) for the oscillator signal to be available
 *                  to the RTCC. The user must make sure that the clock is actually running using RtccGetClkStat()
 *                  before expecting the RTCC to count.
 *
 * Example:     rtccDate dt; dt.wday=05; dt.mday=0x28; dt.mon=0x2; dt.year=0;
 *          rtccTime tm; tm.sec=0x15; tm.min=0x30; tm.hour=01;
 *          rtccRes res=RtccOpen(tm.l, dt.l, 10);
 *                  or
 *          rtccRes res=RtccOpen(0x01301500, 0x00022805, 10);
 ********************************************************************/
extern inline rtccRes __attribute__((always_inline)) 
RtccOpen(rtccClockSel clockSel,unsigned long tm, unsigned long dt, int drift)
{
unsigned int ulDrift=drift<<_RTCCON_CAL_POSITION;

    if( (clockSel == RTCC_LPRC) || (clockSel == RTCC_SOSC) )
    {
        INTEnable(INT_SOURCE_RTCC, INT_DISABLED);	// disable RTCC interrupts
        // be sure the SOSC is enabled, enable RTCC writes
        if( clockSel == RTCC_SOSC )
        {
            mSysUnlockOpLock((OSCCONSET=_OSCCON_SOSCEN_MASK, RTCCONSET=_RTCCON_RTCWREN_MASK));
        }
        else
        {
            mSysUnlockOpLock((RTCCONSET=_RTCCON_RTCWREN_MASK));
        }
        _RtccStop();
        RTCALRMCLR=_RTCALRM_ALRMEN_MASK;// disable the alarm
        INTClearFlag(INT_SOURCE_RTCC);		// clear the interrupt flag
        RTCTIME=tm;
        RTCDATE=dt;
        RTCCONCLR=_RTCCON_CAL_MASK;
        RTCCONSET=ulDrift;
        RTCCONSET=_RTCCON_ON_MASK;
        RtccClockSelect(clockSel);
        RtccWrEnable(0);
        RtccOutputEnable(0);
    }
	return RtccGetClkStat();
}

/*********************************************************************
 * Function:        void RtccShutdown(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function shutdowns the RTCC device. It stops the RTCC clock,
 *                  sets the RTCC Off and disables RTCC write. Disables the Alarm and the OE.
 *                  Clears the alarm interrupt flag and disables the alarm interrupt.
 *
 * Note:            None
 *
 * Example:         RtccShutdown();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccShutdown(void)
{

	INTEnable(INT_SOURCE_RTCC, INT_DISABLED);	// disable RTCC interrupts
	// disable the SOSC, disable RTCC writes
	mSysUnlockOpLock((OSCCONCLR=_OSCCON_SOSCEN_MASK, RTCCONSET=_RTCCON_RTCWREN_MASK));
	_RtccStop();			// make sure RTCC clock is stopped
	RTCALRMCLR=_RTCALRM_ALRMEN_MASK;		// disable the alarm
	INTClearFlag(INT_SOURCE_RTCC);		// clear the interrupt flag
	RtccWrEnable(0);
	RtccOutputEnable(0);
}


// time and alarm functions

/*********************************************************************
 * Function:        void RtccSetTime(unsigned long tm)
 *
 * PreCondition:    tm an unsigned long containing the fields of a valid rtccTime structure:
 *                      - sec:  BCD codification, 00-59
 *                      - min:  BCD codification, 00-59
 *                      - hour: BCD codification, 00-24
 *
 * Input:           tm - the time value to be set
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the current time in the RTCC device.
 *
 * Note:            - The write is successful only if Wr Enable is set.
 *                  The function will enable the write itself, if needed.
 *                  - The device could be stopped in order
 *                  to safely perform the update of the RTC time register.
 *                  However, the device status will be restored but the routine won't wait
 *                  for the CLK to be running before returning. User has to check RtccGetClkStat() (will take approx 30us).
 *                  - The routine could disable the interrupts for a very short time to be able
 *                  to update the time and date registers.
 *
 * Example:         rtccTime tm; tm.sec=0x15; tm.min=0x30; tm.hour=01; RtccSetTime(tm.l);
 *                  or
 *                  RtccSetTime(0x01301500);
 *
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetTime(unsigned long tm)
{

	_RtccSetTimeAndDate((rtccTime*)&tm, 0);
}

/*********************************************************************
 * Function:        unsigned long RtccGetTime(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          The current value of the time which can be safely casted to an rtccTime structure.
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current time of the RTCC device.
 *
 * Note:            - The function makes sure that the read value is valid.
 *                  It avoids waiting for the RTCSYNC to be clear by
 *                  performing successive reads.
 *
 * Example:         rtccTime tm; tm.l=RtccGetTime();
 ********************************************************************/
extern inline unsigned long __attribute__((always_inline)) 
RtccGetTime(void)
{
unsigned int t0, t1;

    do
    {
        t0=RTCTIME;
        t1=RTCTIME;
    }while(t0!=t1);
    return t0;
}

/*********************************************************************
 * Function:        void RtccSetDate(unsigned long dt)
 *
 * PreCondition:    dt is an unsigned long conatining the valid fields of a rtccDate structure:
 *                      - wday: BCD codification, 00-06
 *                      - mday: BCD codification, 01-31
 *                      - mon: BCD codification, 01-12
 *                      - year: BCD codification, 00-99
 *
 * Input:           dt - the date value to be set
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the current date in the RTCC device.
 *
 * Note:            - The write is successful only if Wr Enable is set.
 *                  The function will enable the write itself, if needed.
 *                  - The device could be stopped in order
 *                  to safely perform the update of the RTC time register.
 *                  However, the device status will be restored but the routine won't wait
 *                  for the CLK to be running before returning. User has to check RtccGetClkStat() (will take approx 30us).
 *                  - The routine could disable the interrupts for a very short time to be able
 *                  to update the time and date registers.
 *
 * Example:         rtccDate dt; dt.wday=05; dt.mday=0x28; dt.mon=0x2; dt.year=0; RtccSetDate(dt.l);
 *                  or
 *                  RtccSetDate(0x00022805);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetDate(unsigned long dt)
{

	_RtccSetTimeAndDate(0, (rtccDate*)&dt);
}

/*********************************************************************
 * Function:        unsigned long RtccGetDate(void)
 *
 * PreCondition:    pDt a valid pointer
 *
 * Input:           None
 *
 * Output:          an unsigned long representing the current date.
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current date of the RTCC device. Can be safely cast into rtccDate.
 *
 * Note:            The function makes sure that the read value is valid.
 *                  It avoids waiting for the RTCSYNC to be clear by
 *                  performing successive reads.
 *
 * Example:         rtccDate dt; dt.l=RtccGetDate();
 ********************************************************************/
extern inline unsigned long __attribute__((always_inline)) 
RtccGetDate(void)
{
unsigned int d0, d1;

    do
    {
        d0=RTCDATE;
        d1=RTCDATE;
    }while(d0!=d1);

    return d0;
}

/*********************************************************************
 * Function:        void RtccSetTimeDate(unsigned long tm, unsigned long dt)
 *
 * PreCondition:    tm a valid rtccTime structure having proper halues:
 *                      - sec:  BCD codification, 00-59
 *                      - min:  BCD codification, 00-59
 *                      - hour: BCD codification, 00-24
 *                  date a valid rtccDate structure having proper values
 *                      - wday: BCD codification, 00-06
 *                      - mday: BCD codification, 01-31
 *                      - mon: BCD codification, 01-12
 *                      - year: BCD codification, 00-99
 *
 * Input:           tm - the time value to be set
 *                  dt - the date value to be set
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the current time and date in the RTCC device.
 *
 * Note:            - The write is successful only if Wr Enable is set.
 *                  The function will enable the write itself, if needed.
 *                  - The device could be stopped in order
 *                  to safely perform the update of the RTC time register.
 *                  However, the device status will be restored but the routine won't wait
 *                  for the CLK to be running before returning. User has to check RtccGetClkStat() (will take approx 30us).
 *                  - The routine could disable the interrupts for a very short time to be able
 *                  to update the time and date registers.
 *
 * Example:         rtccTime tm; tm.sec=0x15; tm.min=0x59; tm.hour=0x23;
 *                  rtccDate dt; dt.wday=05; dt.mday=0x28; dt.mon=0x2; dt.year=0;
 *                  RtccSetTimeDate(tm, dt);
 *                  or
 *                  RtccSetTimeDate(0x23591500, 0x00022805);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetTimeDate(unsigned long tm, unsigned long dt)
{

	_RtccSetTimeAndDate((rtccTime*)&tm, (rtccDate*)&dt);
}

/*********************************************************************
 * Function:        void RtccGetTimeDate(rtccTime* pTm, rtccDate* pDt)
 *
 * PreCondition:    pTm, pDt valid pointers
 *
 * Input:           pTm - pointer to a rtccTime union to store the current time
 *                  pDt - pointer to a rtccDate union to store the current date
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function updates the user supplied union/structures with
 *                  the current time and date of the RTCC device.
 *
 * Note:            - The function makes sure that the read value is valid.
 *                  It avoids waiting for the RTCSYNC to be clear by
 *                  performing successive reads.
 *
 * Example:         rtccTime tm; rtccDate dt; RtccGetTimeDate(&tm, &dt);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccGetTimeDate(rtccTime* pTm, rtccDate* pDt)
{
rtccTime t0;
rtccDate d0;

    do
    {
        d0.l=RTCDATE;
        t0.l=RTCTIME;
        pTm->l=RTCTIME;
        pDt->l=RTCDATE;
    }while((d0.l!=pDt->l) || (t0.l!=pTm->l));       // update the user requested data
}

/*********************************************************************
 * Function:        void RtccSetAlarmTime(unsigned long tm)
 *
 * PreCondition:    tm a valid rtccTime structure having proper values:
 *                      - sec:  BCD codification, 00-59
 *                      - min:  BCD codification, 00-59
 *                      - hour: BCD codification, 00-24
 *
 * Input:           tm - the alarm time to be set
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the current alarm time in the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the ALRMTIME register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         rtccTime tm; tm.sec=0x15; tm.min=0x59; tm.hour=0x23; RtccSetAlarmTime(tm.l);
 *                  or
 *                  RtccSetAlarmTime(0x23591500);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetAlarmTime(unsigned long tm)
{

	_RtccSetAlrmTimeAndDate((rtccTime*)&tm, 0);
}

/*********************************************************************
 * Function:        unsigned long RtccGetAlarmTime(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          the current alarm time
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current alarm time of the RTCC device.
 *
 * Note:            None
 *
 * Example:         rtccTime tm; tm.l=RtccGetAlarmTime();
 ********************************************************************/
extern inline unsigned long __attribute__((always_inline)) 
RtccGetAlarmTime(void)
{

    return  ALRMTIME;
}

/*********************************************************************
 * Function:        void RtccSetAlarmDate(unsigned long dt)
 *
 * PreCondition:    dt a valid rtccDate formatted structure having proper values:
 *                      - wday: BCD codification, 00-06
 *                      - mday: BCD codification, 01-31
 *                      - mon: BCD codification, 01-12
 *
 * Input:           dt - value of the alarm date
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the alarm date in the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the ALRMDATE register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *                  - Note that the alarm date does not contain a year field.
 *
 * Example:         rtccDate dt; dt.wday=0; dt.mday=0x12; dt.mon=0x12; RtccSetAlarmDate(dt.l);
 *                  or
 *                  RtccSetAlarmDate(0x121200);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetAlarmDate(unsigned long dt)
{

	_RtccSetAlrmTimeAndDate(0, (rtccDate*)&dt);
}

/*********************************************************************
 * Function:        unsigned long RtccGetAlarmDate(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          The current alarm date. Can be safely cast into an rtccDate.
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current alarm date of the RTCC device.
 *
 * Note:            None
 *
 * Example:         rtccDate dt; dt.l=RtccGetAlarmDate();
 ********************************************************************/
extern inline unsigned long __attribute__((always_inline)) 
RtccGetAlarmDate(void)
{

    return  ALRMDATE;
}

/*********************************************************************
 * Function:        void RtccSetAlarmTimeDate(unsigned long tm, unsigned long dt)
 *
 * PreCondition:    tm a valid rtccTime structure having proper values:
 *                      - sec:  BCD codification, 00-59
 *                      - min:  BCD codification, 00-59
 *                      - hour: BCD codification, 00-24
 *                  dt a valid rtccDate structure having proper values:
 *                      - wday: BCD codification, 00-06
 *                      - mday: BCD codification, 01-31
 *                      - mon: BCD codification, 01-12
 *
 * Input:           tm - the alarm time to be set
 *                  dt - the alarm date to be set
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the current alarm time and date in the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the ALRMTIME, ALRMDATE registers.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *                  - Note that the alarm time does not contain a year field.
 *
 * Example:         rtccTime tm; tm.sec=0; tm.min=0x59; tm.hour-0x23;
 *                  rtccDate dt; dt.wday=0; dt.mday=0x12; dt.mon=0x12;
 *                  RtccSetAlarmTimeDate(tm.l, dt.l);
 *                  or
 *                  RtccSetAlarmTimeDate(0x235900, 0x121200);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetAlarmTimeDate(unsigned long tm, unsigned long dt)
{

	_RtccSetAlrmTimeAndDate((rtccTime*)&tm, (rtccDate*)&dt);
}

/*********************************************************************
 * Function:        void RtccGetAlarmTimeDate(rtccTime* pTm, rtccDate* pDt)
 *
 * PreCondition:    pTm, pDt valid pointers
 *
 * Input:           pTm - pointer to a rtccTime union to store the alarm time
 *                  pDt - pointer to a rtccDate union to store the alarm date
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function updates the user supplied union/structures with
 *                  the current alarm time and date of the RTCC device.
 *
 * Note:            None
 *
 * Example:         rtccTime tm; rtccDate dt; RtccGetAlarmTimeDate(&tm, &dt);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccGetAlarmTimeDate(rtccTime* pTm, rtccDate* pDt)
{

    pTm->l=ALRMTIME;
    pDt->l=ALRMDATE;
}

/*********************************************************************
 * Function:        void RtccAlarmEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function enables the alarm of the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccAlarmEnable();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccAlarmEnable(void)
{

	_RtccWriteInAlarmWindow(&RTCALRMSET, _RTCALRM_ALRMEN_MASK, 0, 0);
}

/*********************************************************************
 * Function:        void RtccAlarmDisable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function disables the alarm of the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccAlarmDisable();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccAlarmDisable(void)
{

	_RtccWriteInAlarmWindow(&RTCALRMCLR, _RTCALRM_ALRMEN_MASK, 0, 0);
}

/*********************************************************************
 * Function:        int RtccGetAlarmEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          curent status of the RTCC alarm
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the alarm status of the RTCC device.
 *
 * Note:            None
 *
 * Example:         int isAlrmEnabled=RtccGetAlarmEnable();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetAlarmEnable(void)
{
int isAlrm0, isAlrm1;

    do
    {
        isAlrm0=RTCALRMbits.ALRMEN;
        isAlrm1=RTCALRMbits.ALRMEN;
    }while(isAlrm0!=isAlrm1);

    return isAlrm0;
}

/*********************************************************************
 * Function:        void RtccChimeEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function enables the chime alarm of the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccChimeEnable();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccChimeEnable(void)
{

	_RtccWriteInAlarmWindow(&RTCALRMSET, _RTCALRM_CHIME_MASK, 0, 0);
}

/*********************************************************************
 * Function:        void RtccChimeDisable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function disables the chime alarm of the RTCC device.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccChimeDisable();
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccChimeDisable(void)
{

	 _RtccWriteInAlarmWindow(&RTCALRMCLR, _RTCALRM_CHIME_MASK, 0, 0);
}

/*********************************************************************
 * Function:        int RtccGetChimeEnable(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          current status of the alarm chime
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the chime alarm of the RTCC device.
 *
 * Note:            None
 *
 * Example:         int isChimeEnabled=RtccGetChimeEnable();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetChimeEnable(void)
{
int ch0, ch1;

    do
    {
        ch0=RTCALRMbits.CHIME;
        ch1=RTCALRMbits.CHIME;
    }while(ch0!=ch1);

    return ch0;
}

/*********************************************************************
 * Function:        void RtccSetAlarmRpt(rtccRepeat rpt)
 *
 * PreCondition:    rpt has to be a proper rtccRepeat enumeration value
 * Input:           rpt         - value of the desired alarm repeat rate
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the RTCC alarm repeat rate.
 *
 * Note:            - The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *
 * Example:         RtccSetAlarmRpt(RTCC_RPT_MIN);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetAlarmRpt(rtccRepeat rpt)
{

	_RtccWriteInAlarmWindow(&RTCALRMCLR, _RTCALRM_AMASK_MASK, &RTCALRMSET, (rpt&0xf)<<_RTCALRM_AMASK_POSITION);
}

/*********************************************************************
 * Function:        rtccRepeat RtccGetAlarmRpt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          The value of the current alarm repeat rate.
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the current RTCC alarm repeat rate.
 *
 * Note:            None
 *
 * Example:         rtccRepeat rptAlrm=RtccGetAlarmRpt();
 ********************************************************************/
extern inline rtccRepeat __attribute__((always_inline)) 
RtccGetAlarmRpt(void)
{
rtccRepeat r0, r1;

    do
    {
        r0=(rtccRepeat)(RTCALRM&_RTCALRM_AMASK_MASK);
        r1=(rtccRepeat)(RTCALRM&_RTCALRM_AMASK_MASK);
    }while(r0!=r1);

    return (rtccRepeat)(r0>>_RTCALRM_AMASK_POSITION);
}

/*********************************************************************
 * Function:        void RtccSetAlarmRptCount(int rptCnt)
 *
 * PreCondition:    rptCnt has to be a value less then 256
 *
 * Input:           rpt         - value of the desired alarm repeat rate
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The function sets the RTCC alarm repeat count.
 *                  The number of alarm triggers will be rptCnt+1:
 *                      - one alarm trigger if rptCnt==0
 *                      - ....
 *                      - 256 alarm triggers if rptCnt=255
 *
 * Note:            - rptCnt will be truncated to fit into 8 bit representation.
 *                  - The function might wait for the proper Alarm window
 *                  to safely perform the update of the RTCALRM register.
 *                  - Interrupts are disabled shortly when properly probing the ALRMSYNC needed.
 *                  - If rptCnt is 0, there will be one alarm trigger.
 *
 * Example:         RtccSetAlarmRptCount(10);
 ********************************************************************/
extern inline void __attribute__((always_inline)) 
RtccSetAlarmRptCount(int rptCnt)
{

	_RtccWriteInAlarmWindow(&RTCALRMCLR, _RTCALRM_ARPT_MASK, &RTCALRMSET, rptCnt&0xff);
}

/*********************************************************************
 * Function:        int RtccGetAlarmRptCount(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          the current alarm repeat count
 *
 * Side Effects:    None
 *
 * Overview:        The function reads the RTCC alarm repeat counter.
 *
 * Note:            The reading is affected by the sttatus of RTCALRM.ALRMSYNC bit.
 *                  Double readings are performed.
 *
 * Example:         int alrmRptCnt=RtccGetAlarmRptCount();
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccGetAlarmRptCount(void)
{
int rpt0, rpt1;

    do
    {
        rpt0=RTCALRMbits.ARPT;
        rpt1=RTCALRMbits.ARPT;
    }while(rpt0!=rpt1);

    return rpt0;
}

/*********************************************************************
 * Function:        int RtccWeekDay(int year, int month, int day)
 *
 * PreCondition:    Date greater than 14 Sep 1752.
 *
 * Input:           year    - year value
 *                  month   - month value, 1-12
 *                  day     - day value, 1-31
 *
 * Output:          the week of the day, 0 for Sun, 1 for Mon and so on
 *
 * Side Effects:    None
 *
 * Overview:        The algorithm calculates the week of the day for new style
 *                  dates, beginning at 14 Sep 1752.
 *                  Based on an algorithm by Lewis Carroll.
 *
 * Note:            None
 *
 * Example:         int weekDay=RtccWeekDay(2004, 02, 28);
 ********************************************************************/
extern inline int __attribute__((always_inline)) 
RtccWeekDay(int year, int month, int day)
{
// month table
int mnthTbl[12]=
{	// Jan, Feb....to Dec
	0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 12
};
int y=year%100;			// just the year, remove the century part
int centItem=(3-((year/100)%4))*2;
int yearItem=y/12+y%12+(y%12)/4;
int wDay=centItem+yearItem+mnthTbl[month-1]+day;

	if( month==1 || month==2 )
	{	// Jan Feb correction, if leap year
		if( (y!=0 && y%4==0) || year%400==0 )
		{
			wDay--;
		}
	}

	return wDay%7;
}

#ifdef	__cplusplus
}
#endif

#endif	/* RTCC_H */

