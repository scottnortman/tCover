//	File:	InputOutput.c

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

	if( PINC & (1<<PC0) )
		return ON;
	else
		return OFF;

}//end GetKeyPos

SWITCH_POS	GetSwitchPos(void){

	//Local Variables
	uint16_t temp;
	
	//Sampl A2D
	INTR_OFF;
	temp = a2dSample(A2D_SWITCH_CH);
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
