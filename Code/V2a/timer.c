/*	File:	timer.c
*	Desc:	This file contains the function
*			implementations for the timer
*			hardware needed on the atmega8.
*	Date:	November 3, 2004
*	Auth:	Scott Nortman
*	Proj:	AutoMotion
*
*	Copywrite (c) 2004 Scott Nortman
*
*	Date		Who				What
*--------------------------------------------------
*	11/3/2004	S. Nortman		Initiated File
*/

#include "includes.h"

void timerInit(void){
/* Desc:	This function initializes the output
*			compare 1 A.  It is set to:
*
*			FAST PWM Mode, rising slope only,
*			set at TOP, clear on compare match.
*
*
*	TIMER INFO:
	TOC1 -> PWM for servo	
*/


//////////////////////////////////////////////////
//	PWM
/////////////////////////////////////////////////

	
	/* Set timer 1 prescale to fclk/8 = 
	*	8.0MHz / 8 = 1.0 Mhz count freq.
	*  Set to mode 14 */
	TCCR1A = 0x82;
	TCCR1B = 0x1A;
	
	/* Load top into ICR1A, 20000 counts */
	ICR1H = TOC1_TOP_VAL >> 8;
	ICR1L = TOC1_TOP_VAL & 0x00FF;
	
	/* Set to 1500 count high time */
	OCR1AH = PWM_DTY_DFLT>>8;
	OCR1AL = PWM_DTY_DFLT&0x00FF;
	
/////////////////////////////////////////////////
//	1 ms timer w/ TOC2
////////////////////////////////////////////////

	//Set TOC2 to mode 2 (CTC), F_OSC/64 count
	TCCR2 |= ( 1<<WGM21 | 1<<CS22 );
	
	//Load OCR2 with the desired TOP value.
	OCR2 = 127;
	
	//Enable compare match interrupt for TOC2
	TIMSK |= ( 1<<OCIE2 );


} /* end initOC1B */
