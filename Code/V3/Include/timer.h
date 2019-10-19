/*	File:	timer.h
*	Desc:	This include file contains the
*			prototypes for the hardware
*			timers on the atmega32.
*			odoModV2 project.
*	Date:	July 16, 2005
*	Auth:	Scott Nortman
*	Proj:	odoModV2
*
*	Copywrite (c) 2005 Bridge Electronic Design, LLC
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
void timerInit		(void);

#endif /* #ifndef TIMER_H */
