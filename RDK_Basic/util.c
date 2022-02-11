/************************************************************************/
/*																		*/
/*	util.c -- Common Utility Procedures for the PIC32MX                 */
/*																		*/
/************************************************************************/
/*	Author: Michael T. Alexander										*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This module contains definitions for common utility functions.      */
/*																		*/
/************************************************************************/
/*  Revision History:						                			*/
/*											                        	*/
/*	04/28/2009 (MichaelA): created			                			*/
/*											                        	*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include "util.h"


/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/***    DelayMs
**
**	Synopsis:
**		DelayMs(tmsDelay)
**
**	Parameters:
**		tmsDelay - the number of milliseconds you want to delay
**
**	Return Values:
**      none
**
**	Errors:
**		none
**
**	Description:
**		This procedure delays program execution for the specified number
**      of miliseconds.
*/
void DelayMs( WORD tmsDelay )
{	
	while ( 0 < tmsDelay ) {
		DelayUs(1000);
		tmsDelay--;
	}
}

/* ------------------------------------------------------------ */
/***    DelayUs
**
**	Synopsis:
**		DelayUs(tusDelay)
**
**	Parameters:
**		tusDelay - the amount of time you wish to delay in microseconds
**
**	Return Values:
**      none
**
**	Errors:
**		none
**
**	Description:
**		This procedure delays program execution for the specified number
**      of microseconds. Please note that the minimal delay supported by
**		this routine is 3 microseconds.
**		
**	Note:
**		This routine is written with the assumption that the
**		system clock is 64 MHz.
*/
void DelayUs( WORD tusDelay )
{
	tusDelay -= 2;
	
    while ( 0 < tusDelay )
    {
	    asm volatile("nop");
        tusDelay--;
    }   // end while
}   // DelayUs

	

//void SetLeds( BYTE stLeds ) {
//	// Set the state of Led1.
//	if ( stLeds & ( 1 << 0 ) ) {
//		prtLed1Set = ( 1 << bnLed1 );
//	}
//	else {
//		prtLed1Clr = ( 1 << bnLed1 );
//	}
//	
//	// Set the state of Led2.
//	if ( stLeds & ( 1 << 1 ) ) {
//		prtLed2Set = ( 1 << bnLed2 );
//	}
//	else {
//		prtLed2Clr = ( 1 << bnLed2 );
//	}
//	
//	// Set the state of Led3.
//	if ( stLeds & ( 1 << 2 ) ) {
//		prtLed3Set = ( 1 << bnLed3 );
//	}
//	else {
//		prtLed3Clr = ( 1 << bnLed3 );
//	}
//	
//	// Set the state of Led4.
//	if ( stLeds & ( 1 << 3 ) ) {
//		prtLed4Set = ( 1 << bnLed4 );
//	}
//	else {
//		prtLed4Clr = ( 1 << bnLed4 );
//	}
//
//}
/*************************************************************************************/
