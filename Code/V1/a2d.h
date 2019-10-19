/*	File:	a2d.h
*	Desc:	This is the include file for the a2d routines for
*			the a2d.c file for the AutoMotion project.
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
*
*	Date		Who				What
*--------------------------------------------------
*	11/3/2004	S. Nortman		Initiated File
*/

#ifndef A2D_H
#define A2D_H

/* includes */
#include "includes.h"

/* defines */


/* prototypes */
void a2dInit(void);
uint16_t a2dSample(uint8_t channel);

#endif /* #ifndef A2D_H */
