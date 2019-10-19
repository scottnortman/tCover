/*	
*	File:	InputOutput.c
*	Desc:	This file contains the IO routines
*			for the tCover microcontroller and the
*			AutoMotion servo controller.
*	Date:	November 3, 2004
*	Auth:	Scott Nortman
*	Proj:	AutoMotion
*
*	Copywrite (c) 2004, 2005, 2006 Scott Nortman and Bridge Electronic
*	Design, LLC, All rights reserved.
*	
*	No portion of this software or any derivative may be used
*	for any purpose without prior written concent from the above
*	author(s).
*
*	REVISION HISTORY
*
*	Current Version:	2.2.1
*
*	Date		Who				What
*	------------------------------------------------------------------------
*	11/3/2004	S. Nortman		Initiated File
*	3/5/06		S. Nortman		Fixed bug w/ ACC input; switch from digital to
*								analog input since the asserted voltage is only
*								2.9V, and the Vmin high from the datasheet is
*								3.0v.
*/

#include "includes.h"

void IOInit(void){

	/* Init HW  Systems*/	
	timerInit();
	a2dInit();
	
	//Turn on PWM Pin
	PWM_ON;
	
	//Set servo to mid position
	SetPWMDuty(PWM_CENTER_DFLT);
	
	//Turn on pull up for MODE pin PB0
	//added 10/14/05, Scott Nortman
	//	pull-up on PB3
	PORTB |= (1<<PB3) ;
	
	//Pull up for acc
	PORTC |= (1<<PC0);
	
	//Pull up for NORM or /REV pin (PD0)
	PORTD |= (1<<PD0);
	
	//Turn on interrupts
	INTR_ON;

}//end IOInit

void SetPWMDuty(uint16_t highTime){

		OCR1AH = highTime>>8;
		OCR1AL = highTime&0x00FF;

} /* end timerSetPWMDuty */

void SetPWMPeriod(uint16_t period){

	ICR1H = period>>8;
	ICR1L = period&0x00FF;

} /* end timerPWMPeriod */

KEY_POS		GetKeyPos	(void){

	//Local variable
	uint16_t temp;
	
	//Sample A2D
	INTR_OFF;
	temp = a2dSample( A2D_ACC_INPUT );
	INTR_ON;
	
	//Return appropriate value based on a2d
	if( temp < A2D_ACC_ON_COUNT )
		return OFF;
	else
		return ON;

//Old code
#if 0
	if( PINC & (1<<PC0) )
		return ON;
	else
		return OFF;
#endif

}//end GetKeyPos

SWITCH_POS	GetSwitchPos(void){

	//Local Variables
	uint16_t temp;
	
	//Sampl A2D
	INTR_OFF;
	temp = a2dSample(A2D_SWITCH);
	INTR_ON;

	//Define switch value based on A2D count
	if		 ( temp < DOWN_MAX_COUNT ){
		return DOWN;
	}//end if
	else if (temp <= UP_MIN_COUNT && temp >= DOWN_MAX_COUNT){
		return CENTER;
	}//end else if
	else{
		return UP;
	}//end else

}//end GetSwitchPos

DIR_PIN		GetDirPin  	(void){

	if( PIND & (1<<PD0) )
		return NORM;
	else
		return REV;

}//end GetDirPin
