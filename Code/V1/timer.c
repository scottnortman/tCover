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

	/* First set the data direction bit
	*	for PORTB, pin 1 to OUT for
	*	servo PWM generation.
	*/
	//sbi(DDRB, PB1);
	DDRB |= (1<<PB1);
	
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
	OCR2 = 31;
	
	//Timer OC2 will now execute at
	//	f = 8e6 / (2*64*32) = ~1.953 kHz.
	
	//Enable compare match interrupt for TOC2
	TIMSK |= ( 1<<OCIE2 );


} /* end initOC1B */

void timerSetPWMDuty(uint16_t highTime){

	OCR1AH = highTime>>8;
	OCR1AL = highTime&0x00FF;

} /* end timerSetPWMDuty */

void timerSetPWMPeriod(uint16_t period){

	ICR1H = period>>8;
	ICR1L = period&0x00FF;

} /* end timerPWMPeriod */

void timerBusyDelay(uint16_t counts){

	uint16_t tempA, tempB;
	
	for(tempA=0;tempA<counts;tempA++)
		for(tempB=0;tempB<counts;tempB++);

}

