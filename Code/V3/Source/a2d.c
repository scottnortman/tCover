/*	File:	a2d.c
*	Desc:	This file contains the function
*			implementations for the a2d
*			hardware needed on the atmega8.
*	Date:	November 3, 2004
*	Auth:	Scott Nortman
*	Proj:	AutoMotion
*
*	Copywrite (c) 2004 Scott Nortman
*
*	No portion of this software or any derivative may be used
*	for any purpose without prior written concent from the above
*	author.
*
*	Date		Who				What
*--------------------------------------------------
*	11/3/2004	S. Nortman		Initiated File
*/

#include "includes.h"

void a2dInit(void){

	/* Set to internal vref = 2.56 v, cap on vref */
	
	/* Turn a2d on, disable int. */
	ADCSRA |= (1<<ADEN | 1<<ADPS2 | 1<<ADPS1);

} /* end a2dInit */

uint16_t a2dSample(uint8_t channel){

	/* set channel */
	ADMUX = ((ADMUX & ~0x07) | (channel & 0x07));
	
	/* trigger conversion */
	ADCSRA |= (1<<ADSC);
	
	/* wait for conversion */
	while( (ADCSRA & (1<<ADSC)) );
	
	return ((uint16_t)(ADCL | ADCH<<8));

} /* end trigger conversion */

