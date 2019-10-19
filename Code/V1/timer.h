/*	File:	timer.h
*	Desc:	This include file contains the
*			prototypes for the hardware
*			timers on the atmega32.
*			odoModV2 project.
*	Date:	August 6, 2004
*	Auth:	Scott Nortman
*	Proj:	odoModV2
*
*	Copywrite (c) 2004 Scott Nortman and John Camp
*
*	Date		Who				What
*--------------------------------------------------
*	8/6/2004	S. Nortman		Initiated File
*/

#ifndef TIMER_H
#define TIMER_H

/* Includes */
#include "includes.h"

/* Definitions */
#define TOC1_TOP_VAL	(20000)
#define PWM_DTY_DFLT	(1500)

/* Function Prototypes */
void timerInit(void);
void timerSetPWMDuty(uint16_t highCount);
void timerSetPWMPeriod(uint16_t period);
void timerBusyDelay(uint16_t counts);

#endif /* #ifndef TIMER_H */
