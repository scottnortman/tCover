//	File:	InputOutput.h

#ifndef INPUTOUTPUT_H
#define INPUTOUTPUT_H

#include "includes.h"

//Macros
//A2dC counts corrosponding to voltages for switch
#define UP_MIN_VOLTS		3.0
#define DOWN_MAX_VOLTS		2.0
#define UP_MIN_COUNT		(uint16_t)((UP_MIN_VOLTS/5.0)*1024.0)
#define DOWN_MAX_COUNT		(uint16_t)((DOWN_MAX_VOLTS/5.0)*1024.0)
//Switch A2D channel
#define A2D_SWITCH_CH		1
#define A2D_SPEED_CH		2
#define A2D_OPEN_CH			3
#define A2D_CLSD_CH			4
//Defines for servo counts
#define PWM_OPEN_LIM		2250							//ISR counts
#define PWM_CLSD_LIM		750								//ISR Counts
#define PWM_OPEN_DFLT		1750
#define PWM_CLOSED_DFLT		1250
#define PWM_CENTER_DFLT		1500
#define PWM_SPEED_DFLT		4						// x 1ms between each servo step
//Turn on/off pwm
#define PWM_ON				(DDRB |= (1<<PB1))
#define PWM_OFF				(DDRB &= ~(1<<PB1))
//added 10/14/05
#define GET_RESET_INPUT		(PINB & (1<<PB3))


//Types
typedef enum{
	DOWN	= 1,
	CENTER	= 2,
	UP		= 3
}SWITCH_POS;

typedef enum{
	ON	= 1,
	OFF	= 2
}KEY_POS;

typedef enum{
	NORM 	= 1,
	REV		= 2
}DIR_PIN;

typedef struct{
	bool		SwitchEventFlag;
	SWITCH_POS	SwitchPosOld;
	SWITCH_POS	SwitchPosNew;
	uint32_t	SwitchTimeNew;
}SWITCH_EVENT_STRUCT;

typedef struct{
	bool		KeyEventFlag;
	KEY_POS		KeyPosOld;
	KEY_POS		KeyPosNew;
	uint32_t	KeyTimeNew;
}KEY_EVENT_STRUCT;

typedef struct{
	uint16_t	UpperLimit;
	uint16_t	LowerLimit;
	uint16_t	Speed;
}SERVO_PARAMS;

//Prototypes
//hardware Setup
void IOInit				(void);
//Servo Control
void SetPWMDuty			(uint16_t	highCount);
void SetPWMPeriod		(uint16_t	period);
//Inputs
KEY_POS		GetKeyPos	(void);
SWITCH_POS	GetSwitchPos(void);
DIR_PIN		GetDirPin	(void);

#endif 	//#ifndef INPUTOUTPUT_H
