/*	File:	includes.h
*	Desc:	This include file contains #includes
*			for all of the *.h files used in the
*			AutoMotion project.
*	Date:	November 3, 2004
*	Auth:	Scott Nortman
*	Proj:	AutoMotion
*
*	Copywrite (c) 2004 Scott Nortman
*
*	Current Version:  v2.2.1
*
*	Date		Who				What
*--------------------------------------------------
*	11/3/2004	S. Nortman		Initiated File
*	3/5/06		S. Nortman		Removed obsolete header file as per new avr-gcc.
*								Updated version number.
*/

#ifndef INCLUDES_H
#define INCLUDES_H

/* sets type of mem to use */
#ifndef PROGMEM
#define PROGMEM		__attribute__ ((progmem))
#endif
#ifndef EEPROM
#define EEPROM 		__attribute__ ((section (".eeprom")))
#endif

/* Atmel specific include files */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <inttypes.h>
#include <stdlib.h>

typedef uint8_t bool;

/* Project related include files */
#include "timer.h"
#include "a2d.h"
#include "InputOutput.h"

/* Project wide definitions */
#define F_OSC		(8.0e6)				/* 8.0 MHz osc. freq. */
#define FALSE		(0)
#define TRUE		(1)
#define INTR_ON		sei()
#define INTR_OFF	cli()

#define LED_ON		sbi(PORTD, PD0)
#define LED_OFF		cbi(PORTD, PD0)



#endif /* #ifndef INCLUDES_H */

