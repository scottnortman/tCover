/*	
*	File:	main.c
*	Desc:	This file contains the main routine
*			for the tCover microcontroller and the
*			AutoMotion servo controller.
*	Date:	November 3, 2004
*	Auth:	Scott Nortman
*	Proj:	AutoMotion
*
*	Copywrite (c) 2004, 2005 Scott Nortman and Bridge Electronic
*	Design, LLC, All rights reserved.
*	
*	No portion of this software or any derivative may be used
*	for any purpose without prior written concent from the above
*	author(s).
*
*	REVISION HISTORY
*
*	Current Version:	1.4
*
*	Date		Who				What
*	------------------------------------------------------------------------
*	11/3/2004	S. Nortman		Initiated File
*	12/20/2004	S. Nortman		Set to Ver. 1.0, added comments, cleaned up
*	02/05/2005	S. Nortman		Added code to delay by 0.5s each change of state,
*								change a2d reading such that it is scaled
*								by 3/4, and changed to V1.1.
*	02/08/2005	s. Nortman		Implemented delay for ACC only, not switch,
*								changed to V1.2
*	03/07/2005	S. Nortman		Fixed bug:  Delay was present when switch
*								transitioned from HIGH or LOW to CENTER, but
*								that wasn't desired.  Added a simple state
*								machine to implement delay ONLY when old and new
*								states were both CENTER; ie, only when swich is
*								constantly at the center position, therefore the
*								desired behavior is waht was expected.  Now the delay
*								only affects changes of the ACC line, not the manual
*								override switch.  Changed to Version 1.3.
*	7/9/2005	S. Nortman		Implemented a new feature to resolve the servo "humming"
*								problem.  Customers were complaining about the fact that
*								the servo was "humming" when at the endpoints of travel.
*								This was due to numerous factors, including mis-
*								alignment of the mechanical linkages, tolerances of
*								the hole locations from user installation error, and
*								the fact that the HiTec HS-322HD servos are a bit
*								more prone to hum when idle.  The solution:  turn off
*								the servo PWM signal, after a short timeout period, by
*								setting the pin to a high impedance state (input).  This
*								effectively "kills" the servo, preventing it from
*								humming.  Changed to Version 1.4 . 
*
*	NOTES:
*
*		The overall purpose of this code is to control the position of a standard RC
*	servo based on numerous inputs.  The inputs are as follows:
*		1.  MODE
*				The mode input is normally pulled up via an internal pull up
*				resistor.  The servo is commanded to move to a high or low limit
*				based on the state of the ACC input and the SWITCH POSITION input.
*				When the SWITCH is in the CENTER position, the servo moves to high
*				or low based on ACC; vice-versa if REV is asserted.  If mode is pulled
*				high, the servo ignores ACC and moves to the high position while the
*				switch is up , the low position while the switch is down, and stops
*				moving while the switch is in the center position.
*				
*		2.  SWITCH POSITION
*				The switch may be in one of three states:  UP, CENTER, or DOWN.
*				If mode is LOW (tCover operation), then the servo does the following:
*				SWITCH = UP 	--> Servo moves to high position
*				SWITCH = CENTER	--> Servo follows ACC
*				SWITCH = DOWN	--> Servo moves to low position
*
*		3.  REV Input
*				Reverses the direction of movement; only used for tCover mode.
*
*		4.  ACC Input
*				Used when MODE is pulled low; if the switch is in the center position
*				ACC controls the position of the servo.
*
*		5.  LO_LIM
*				Defines the PWM duty cycle of the low limit servo position, based on a
*				fixed offset value added to the a2d count.
*
*		6.  HI_LIM
*				Defines the PWM duty cycle of the high limit servo position, based on a
*				fixed offset added to the a2d count.
*
*		7.  SPEED input
*				Defines the number of ISR iterations in between each change in the duty
*				cycle of the servo.  Higher to +5V increments the ISR counts, slowing
*				the servo down.  Lower to 0V decrements the ISR counts, increasing the
*				speed of the servo.
*
*	ADDITIONAL NOTES (added for V 1.4 )
*
*		The servo PWM signal is turned off after a small delay to prevent the servo from
*		"humming" at the endpoints of travel.  This is implemented by setting the OC1A
*		output pin to a high impedance input, effectively cutting off the PWM to the servo.
*
*/

/* Includes */
#include "includes.h"

//Possible switch states
#define DOWN	1
#define CENTER	2
#define UP		3

//Possible states of cover/servo
#define STOP	0
#define CLOSED	1
#define OPEN	2

//A2D Channels
#define A2D_SWITCH		1
#define A2D_SPEED		2
#define A2D_HIGH_LIM	3
#define A2D_LOW_LIM		4

//A2dC counts corrosponding to voltages for switch
#define UP_MIN_VOLTS	3.0
#define DOWN_MAX_VOLTS	2.0
#define UP_MIN_COUNT	(uint16_t)((UP_MIN_VOLTS/5.0)*1024.0)
#define DOWN_MAX_COUNT	(uint16_t)((DOWN_MAX_VOLTS/5.0)*1024.0)

//Defines for servo counts
#define PWM_COUNTS_OPEN		2250
#define PWM_COUNTS_CLOSED	750
#define PWM_DEFAULT			1500

//Defines for NORM or REVERSE rotation
#define NORM		1 	//pulled up internally
#define REV			0	//pulled to GND

//Added for filter timeout value so the servo
//	doesn't appear to "jump" when the ignition
//	key sweeps past the ACC position as the user
//	turns on thier car.
#define FILTER_TIMEOUT		2000	//counts of ISR

//Timeout amount that specifies how many ISR iterations
//	to wait until the PWM signal is stopped.  This
//	will prevent the servo from humming after the
//	specified timeout period.
#define HUM_TIMEOUT			5000	// # ISR iterations
#define HUM_THRESHOLD		4		//a2d counts

//Static globals	
//Desired servo state, set in main loop, used in ISR
static	uint8_t g_servoStateDesired;
static uint8_t switchPosNew;		//Holds the state of the switch

//Main Routine
int16_t main( void ){
/*	Desc:		This is the main routine for the tCover servo control module.
*	Args:		None.
*	Ret:		int16_t, so compiler doesn't complain.
*	Globals:	g_servoStateDesired
*	PreReq:		None.
*	Side E:		A2D inputs are sampled, PWM is generated.
*	Notes:		None.
*/

	//Inputs

	uint8_t 	accSignal;		//Holds the state of the accessory signal
	uint8_t 	modePin;		//Holds the state of the MODE input
	uint8_t	dirPin;			//Hold the state of the direction pin.
	uint16_t	temp;
	
	/* Init HW  Systems*/	
	timerInit();
	a2dInit();
	
	//Set servo to mid position
	timerSetPWMDuty(PWM_DEFAULT);
	
	//Turn on pull up for MODE pin PB0
	//sbi(PORTB, PB0);
	PORTB |= (1<<PB0);
	
	//Pull up for acc
	//sbi(PORTC, PC0);
	PORTC |= (1<<PC0);
	
	//Pull up for NORM or /REV pin (PD0)
	//sbi(PORTD, PD0);
	PORTD |= (1<<PD0);
	
	//Turn on interrupts
	INTR_ON;
	
	for(;;){
	
		//Get state of inputs
		//MODE pin
		modePin		= (PINB & (1<<PB0) )>>PB0;
		//ACC signal
		accSignal	= (PINC & (1<<PC0) )>>PC0;
		//NORM or REV
		dirPin		= (PIND & (1<<PD0)	)>>PD0;
		//SWITCH position
		INTR_OFF;
		temp = a2dSample(A2D_SWITCH);
		INTR_ON;
		
		//Define switch value based on A2D count
		if		 ( temp < DOWN_MAX_COUNT ){
			switchPosNew = DOWN;
		}//end if
		else if (temp <= UP_MIN_COUNT && temp >= DOWN_MAX_COUNT){
			switchPosNew = CENTER;
		}//end else if
		else{
			switchPosNew = UP;
		}//end else
			
		//Control servo based on inputs
		if		( switchPosNew == UP){
		
			//Switch is in UP position, set servo to OPEN position
			g_servoStateDesired  = OPEN;
		
		}//end UP
		else if(switchPosNew == DOWN){
		
			//Switch is in DOWN position, set servo to CLOSE position
			g_servoStateDesired = CLOSED;

		}//end DOWN
		else if(switchPosNew == CENTER){
		
			//Switch is in CENTER position, check MODE and ACC to see
			//	what to do:
			//
			//	Truth Table
			//	MODE	ACC		SERVO
			//	0		0		CLOSED
			//	0		1		OPEN
			//	1		0		STOP
			//	1		1		STOP
			if(modePin){
			
				//MODE is true, so do not return servo to CLOSED,
				//	leave it at its current position.
				g_servoStateDesired = STOP;
			
			}//end if
			else{
			
				//MODE is false, see what ACC is
				if(accSignal){
				
					//Handle NORM or /REV
					if(dirPin == NORM)
						g_servoStateDesired = OPEN;
					else
						g_servoStateDesired = CLOSED;
				}//end if(accSignal)
				else{
				
					if(dirPin == NORM)
						g_servoStateDesired = CLOSED;
					else
						g_servoStateDesired = OPEN;	
				}//end else
				
			}//end else
		
		}//end CENTER
		
	}//end for(;;)

	//So compiler doesn't complain
	return( 0 );

} /* end main */


//Interrupt service routine for TOC2 compare match
SIGNAL(SIG_OUTPUT_COMPARE2){
/*	Desc:		This is the interrupt routine for the tCover servo control module.
*				It runs on a successful compare match of OC2.
*	Args:		None.
*	Ret:		None.
*	Globals:	g_servoStateDesired
*	PreReq:		OC2 interrupts need to be configured for this ISR to run.
*	Side E:		PWM Changes.
*	Notes:		None.
*/

	//Local variables
				uint16_t 	servoClosedCount;
				uint16_t 	servoOpenCount;
	static		uint16_t	servoActualCount = PWM_DEFAULT;
	static		uint8_t	counter;
	static		uint8_t	servoStateActual;
	static		uint16_t	stateTimer;
	static		uint8_t	switchPosOld;
	//Added for hum timeout
	static		uint16_t	humTimer;
	
	if(switchPosNew == CENTER && switchPosOld == CENTER){
	
		if	 (servoStateActual != g_servoStateDesired){
		
			stateTimer++;
			
			if(stateTimer > FILTER_TIMEOUT)
				servoStateActual = g_servoStateDesired;
		
		}
		else{
		
			stateTimer = 0;
		
		}
		
	}
	else{
		servoStateActual = g_servoStateDesired;
	}

	//Check to see there is a count to decrement
	if(counter)
		//If so, decrement
		counter--;
	//Otherwise, we are zero...
	else{
	
		//Grab the A2D value, and assign it to counter.
		//	Note:  The higher the voltage, the slower
		//	the servo moves.
		counter = a2dSample(A2D_SPEED)>>6;
		
		//Get endpoint values from the potentiometers
		//	For the LOW LIM setting, the travel is INC as V increases.
		//	For the HIGH LIM, the travel is DEC. as V increases.
		servoClosedCount 	= PWM_COUNTS_CLOSED + 3*(a2dSample(A2D_LOW_LIM)>>2);
		servoOpenCount 		= PWM_COUNTS_OPEN - 3*(a2dSample(A2D_HIGH_LIM)>>2);
		
		//See what we need to do with the servo
		if			(servoStateActual == OPEN){
		
			//Servo needs to be at the OPEN position, or newMax.
			//	Compare the current position of the servo to newMax.
			//  If there is a difference, add a value (determined by
			//	the speed setting) and wait until next iteration to
			//	add more.
			if		(servoActualCount < servoOpenCount){
			
				//If we are in this statement, the servoActualCount != servoOpenCount,
				//	so we need to adjust the servoActualCount as needed to get the
				//	value to match servoOpenCount.
				servoActualCount++;
				
				//humTimer = HUM_TIMEOUT;
			
			}//end if
			else if(servoActualCount > servoOpenCount){
				
				//If we are in this statement, the servoActualCount != servoOpenCount,
				//	so we need to adjust the servoActualCount as needed to get the
				//	value to match servoOpenCount.
				servoActualCount--;
				
				//humTimer = HUM_TIMEOUT;
			
			}//end else if
			
			if( abs( servoActualCount - servoOpenCount ) > HUM_THRESHOLD ){
			
				humTimer = HUM_TIMEOUT;
			
			}
		
		}//end if
		else if(servoStateActual == CLOSED){
		
			if(servoActualCount > servoClosedCount){
			
				//If we are in this statement, the servoActualCount != servoOpenCount,
				//	so we need to adjust the servoActualCount as needed to get the
				//	value to match servoClosedCount.
				servoActualCount--;
				
				//humTimer = HUM_TIMEOUT;

			}//end if
			else if(servoActualCount < servoClosedCount){
			
				//If we are in this statement, the servoActualCount != servoOpenCount,
				//	so we need to adjust the servoActualCount as needed to get the
				//	value to match servoClosedCount.
				servoActualCount++;
				
				//humTimer = HUM_TIMEOUT;
				
			}//end else if
			
			if( abs( servoActualCount - servoClosedCount ) > HUM_THRESHOLD ){
			
				humTimer = HUM_TIMEOUT;
			
			}
		
		}//end else if
		
		
		if( humTimer ){
		
			DDRB |= (1<<PB1);
		
			//Set the duty cycle.
			timerSetPWMDuty(servoActualCount);
			
			humTimer--;
		
		}
		else
			DDRB &= ~(1<<PB1);
			
		
	
	}//end else
	
	//Update states
	switchPosOld = switchPosNew;

}//end SIG_OUTPUT_COMPARE0
