/*	
*	File:	main.c
*	Desc:	This file contains the main routine
*			for the tCover microcontroller and the
*			AutoMotion servo controller.
*	Date:	December 17, 2005
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
*	Current Version:	3.0.0
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
*	7/16/2005	S.Nortman		Completely rewrote all main software.  This was to allow
*								the use of an internal state machine to permit the user to
*								adjust all parameters via the over ride switch and ignition
*								key. Changed to  v2.0.0
*	10/14/05	S. Nortman		Implemented the following changes:
*								(1)  Increased the time allowed to enter program mode, from
*									~5.5 second timeout (5500 MS_TIMER counts) to ~ 8 seconds
*									( 8000 MS_TIMER counts ) .
*								(2)  Increased the time allowed to enter the locked mode, from
*									~2 seconds ( 2000 MS_TIMER counts ) to ~ 3 seconds ( 3000
*									MS_TIMER counts) .
*								(3) Implemented external reset ability.  A short circuit on
*									Pin 5 of JP4 ( PORTC, PIN6 ) to GND for more then
*									~100 ms will cause the tCover to reset all EEPROM values
*									and then jump into STATE_REBOOT which will reset all 
*									variables, and contine  on to STATE_NORMAL.
*								Changed to version v2.1.0
*
*	12/17/2005	S. Nortman		Rewrote main software for the third revision.  Removed the
*								program state machine used in v2 for setting the upper, lower,
*								and speed settings.  Now the three parameters are set using
*								the potentiometers as with the version 1 firmware.
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

//Defines for NORM or REVERSE rotation
#define NORM		1 	//pulled up internally
#define REV			0	//pulled to GND
//Sample rate = SAMPLE_DIV * 1 ms
#define SAMPLE_DIV			20
//Number of elements in filter array
#define	FILTER_SIZE			3
#define HUM_TIMEOUT			3000	//3 sec. timeout
#define ACC_TIMEOUT			500
//Defines for edges
#define	POSEDGE				1
#define NEGEDGE				2
//UP to CENTER switch count to set STATE_LOCKED
#define LOCKED_CNT_REQ		4
//Timeout period to enter STATE_LOCKED
#define	LOCKED_TIMEOUT		4000	//~ 4 seconds//changed 12/17/2005
//Number of counts to enter STATE_DEMO
#define	DEMO_CNT_REQ		5
//Timeout to enter / exit STATE_DEMO
#define DEMO_TIMEOUT		5000	//~ 5 seconds
#define	DEMO_CYCLE_TIME		10000
#define DEMO_SPEED			40
#define PWM_ADJ_RESOLUTION	10


//State machine enumerations
typedef enum{

	STATE_REBOOT	= 0,
	STATE_NORMAL	= 1,
	STATE_LOCKED	= 2,
	STATE_DEMO		= 3

}STATE;

//Ram Based
static SERVO_PARAMS ServoParamsRam = {
	PWM_OPEN_DFLT,
	PWM_CLOSED_DFLT,
	PWM_SPEED_DFLT
};
static SERVO_PARAMS *ServoParamsRamPtr = &ServoParamsRam;

//32 bit ms counter
static volatile uint32_t	MS_TIMER;	
//Flag indicating we ned to sample the pin
static volatile bool		SampleFlag;
//Vaiable to hold the current state
static	STATE				CurrentState;
//Holds servo position
static uint16_t			DesiredDutyCycle;
static volatile uint16_t	CurrentDutyCycle;

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
	
	//Local variables
	//For switch input
	SWITCH_POS			SwitchPosArray[FILTER_SIZE];
	SWITCH_POS			SwitchPosOld;
	SWITCH_POS			SwitchPosNew;						//Holds value of switch position
	SWITCH_EVENT_STRUCT	SwitchEventStruct;
	//For Key Input
	KEY_POS				KeyPosArray[FILTER_SIZE];
	KEY_POS				KeyPosOld;
	KEY_POS				KeyPosNew;							//hold value of key position
	KEY_EVENT_STRUCT	KeyEventStruct;
	
	//StateNormal variables
	//Variable to hold STATE_NORMAL open timeout info
	uint32_t			StateNormalOpenTime;
	//Sub-state variable indicating active edge
	uint8_t			StateNormalEdge;
	//Variable to hold override switch transitions for lock mode
	uint8_t			StateNormalLockCount;
	//Variable to hold timeout for lock mode
	uint32_t			StateNormalLockTime;
	//Switch transitions for demo mode
	uint8_t			StateNormalDemoCount;
	//Timeout for demo mode
	uint32_t			StateNormalDemoTime;
	
	//STATE_LOCKED variables
	bool				StateLockedInit;
	//Counts switch DOWN to CENTER edges
	uint8_t			StateLockedEdgeCount;
	//Holds timeout value
	uint32_t			StateLockedLockTime;
	
	//STATE_DEMO variables
	uint8_t			StateDemoEdgeCount;
	uint32_t			StateDemoTimeout;
	bool				StateDemoCycleFlag;
	uint32_t			StateDemoCycleTime;
	bool				StateDemoInit;
	uint16_t			StateDemoNormalSpeed;
	
	//For user implemented reset
	uint32_t			UserReset;

	
	//Initialize global varaibles
	//PWM Values
	CurrentDutyCycle = PWM_CENTER_DFLT;
	DesiredDutyCycle = PWM_CENTER_DFLT;
	//Init flags
	SampleFlag = FALSE;
	//Set state
	CurrentState = STATE_REBOOT;
	//Misc. Inits
	StateNormalEdge = POSEDGE;
	
	//Init HW
	IOInit();

	//Start main infinite loop
	for(;;){
	
		//Check sample flag
		if(SampleFlag){
		
			//Reset Flag
			SampleFlag = FALSE;
			
			//Shift and sample Data
			//Override switch
			SwitchPosArray[2] = SwitchPosArray[1];
			SwitchPosArray[1] = SwitchPosArray[0];
			SwitchPosArray[0] =	 GetSwitchPos();
			//Ignition Key
			KeyPosArray[2] = KeyPosArray[1];
			KeyPosArray[1] = KeyPosArray[0];
			KeyPosArray[0] = GetKeyPos();
			
			//Assign new value based on inputs after debounce filtering
			//Check switch input
			if( 	(SwitchPosArray[2] == SwitchPosArray[1])
				&&	(SwitchPosArray[1] == SwitchPosArray[0]) ){
				
				//All of the inputs are the same, assume it has settled
				SwitchPosNew = SwitchPosArray[0];
				
			}//end if
			//Check ignition key input
			if(		(KeyPosArray[2] == KeyPosArray[1])
				&&	(KeyPosArray[1] == KeyPosArray[0]) ){
				
				//All of the inputs are the same, assume it has settled
				KeyPosNew = KeyPosArray[0];
				
			}//end if		
			
			if( SwitchPosOld != SwitchPosNew ){
				
				//There was a state change on the input
				SwitchEventStruct.SwitchPosNew 	= SwitchPosNew;
				SwitchEventStruct.SwitchPosOld 	= SwitchPosOld;
				INTR_OFF;
				SwitchEventStruct.SwitchTimeNew	= MS_TIMER;
				INTR_ON;
				
				//Set the flag
				SwitchEventStruct.SwitchEventFlag = TRUE;
			
			}//end if Old != New
			
			if( KeyPosOld != KeyPosNew ){
				
				//There was a state change on the input
				KeyEventStruct.KeyPosNew	= KeyPosNew;
				KeyEventStruct.KeyPosOld	= KeyPosOld;
				INTR_OFF;
				KeyEventStruct.KeyTimeNew	= MS_TIMER;
				INTR_ON;
				
				//Set the flag
				KeyEventStruct.KeyEventFlag = TRUE;
	
			}//end if Old != New
			
			//Update old values
			SwitchPosOld	= SwitchPosNew;
			KeyPosOld		= KeyPosNew;
			
			//Added 10/14/05, Scott Nortman
			//Check the state of the user reset pin.
			//	if asserted for > USER_RESET_TIMEOUT
			//	reset the EEPROM values from FLASH, and
			//	go into STATE_REBOOT.
			if( !GET_RESET_INPUT && !UserReset ){
			
				//user reset pin is asserted;
				UserReset = TRUE;
				
					
				CurrentState = STATE_REBOOT;
							
			}//end if
			
			//Sample analog inputs
			ServoParamsRamPtr->UpperLimit 	= PWM_OPEN_LIM - 3*(a2dSample(A2D_OPEN_CH)>>2);
			ServoParamsRamPtr->LowerLimit 	= PWM_CLSD_LIM + 3*(a2dSample(A2D_CLSD_CH)>>2);
			ServoParamsRamPtr->Speed		= a2dSample(A2D_SPEED_CH)>>4;
			
		}//end if(SampleFlag)
		
		
		//State Machine
		if		(CurrentState == STATE_REBOOT){
		
			//Initialize Input states
			SwitchEventStruct.SwitchPosOld 		= GetSwitchPos();
			SwitchEventStruct.SwitchPosNew 		= SwitchEventStruct.SwitchPosOld;
			SwitchEventStruct.SwitchEventFlag 	= TRUE;
			INTR_OFF;
			SwitchEventStruct.SwitchTimeNew		= MS_TIMER;
			INTR_ON;
			
			KeyEventStruct.KeyPosOld			= GetKeyPos();
			KeyEventStruct.KeyPosNew			= KeyEventStruct.KeyPosOld;
			KeyEventStruct.KeyEventFlag 		= TRUE;
			INTR_OFF;
			KeyEventStruct.KeyTimeNew			= MS_TIMER;
			INTR_ON;

			//Init values
			//STATE_NORMAL variables
			StateNormalOpenTime		= 0;
			StateNormalEdge			= POSEDGE;
			StateNormalLockCount	= 0;
			StateNormalLockTime		= 0;
			StateNormalDemoCount	= 0;
			StateNormalDemoTime		= 0;
			//STATE_LOCKED
			StateLockedInit			= FALSE;
			StateLockedEdgeCount	= 0;
			StateLockedLockTime		= 0;
			//STATE_DEMO
			StateDemoEdgeCount		= 0;
			StateDemoTimeout		= 0;
			StateDemoCycleFlag		= FALSE;
			StateDemoCycleTime		= 0;
			StateDemoInit			= FALSE;
			//User reset
			UserReset				= FALSE;
			
			ServoParamsRamPtr->UpperLimit 	= PWM_OPEN_LIM - 3*(a2dSample(A2D_OPEN_CH)>>2);
			ServoParamsRamPtr->LowerLimit 	= PWM_CLSD_LIM + 3*(a2dSample(A2D_CLSD_CH)>>2);
			ServoParamsRamPtr->Speed		= a2dSample(A2D_SPEED_CH)>>4;
			
			//Initialize watchdog timer for 500 ms timeout
			wdt_enable(WDTO_500MS);
			
			//Set next state to NORMAL
			CurrentState = STATE_NORMAL;
			
		}//end STATE_REBOOT
		else if(CurrentState == STATE_NORMAL){
		/*	Note:  Within this "Superstate" there
		*	are substates.
		*
		*/
		
			//Handle NORMAL operation
			//Combinatorial events
			if		(SwitchPosNew == UP){
			
				//Set open duty cycle
				//INTR_OFF;
				//DesiredDutyCycle = ServoParamsRamPtr->UpperLimit;
				//INTR_ON;
				SetPWMDuty(2000);
			}
			else if(SwitchPosNew == DOWN){
				
				//Set servo to lower limit
				//INTR_OFF;
				//DesiredDutyCycle = ServoParamsRamPtr->LowerLimit;
				//INTR_ON;	
				SetPWMDuty(1000);
			}//end DOWN
			else if(SwitchPosNew == CENTER){
				
				//Check for open timeout
				INTR_OFF;
				if( 	((MS_TIMER - StateNormalOpenTime) > ACC_TIMEOUT)
					&&	( KeyEventStruct.KeyPosNew == ON ) ){
				
					DesiredDutyCycle = ServoParamsRamPtr->UpperLimit;
	
				}
				INTR_ON;
				
			}//end SwitchPosNew == CENTER
			
			//For lock mode, the ignition key must be on, and we must get the
			//	proper edges from the over ride switch:  Four UP to CENTER
			//	transitions in under 3 seconds will place the cover in lock
			//	mode.
			if( 0){//KeyPosNew == ON ){
		
				//Look for edges to put cover in the lock mode
				if (SwitchEventStruct.SwitchEventFlag){
				
					//Look for UP to CENTER switch transitions
					if(		(SwitchEventStruct.SwitchPosOld == UP)
						&&	(SwitchEventStruct.SwitchPosNew == CENTER) ){
						
						if( !StateNormalLockCount ){
						
							//The count is zero, so grab the starting time;
							//Grab Starting time
							StateNormalLockTime = SwitchEventStruct.SwitchTimeNew;
							
							//Increment count
							StateNormalLockCount++;
						
						}//end !StateNormalLockCount
						else{
						
							//We have a non-zero count value, so keep incrementing
							StateNormalLockCount++;
							
							//See if we've reached the needed counts within the time limit
							if(		( StateNormalLockCount >= LOCKED_CNT_REQ )
								&&	( ( SwitchEventStruct.SwitchTimeNew - StateNormalLockTime ) <= LOCKED_TIMEOUT ) ){
								
								//We've reached the reqired counts within the timeout
								//clear count
								StateNormalLockCount = 0;
								//set new state
								CurrentState = STATE_LOCKED;
								
							}//end if
						
						}//end else
						
					}//end UP to CENTER
					//We also need to check for STATE_DEMO by watching CENTER to DOWN transitions
					//To enter this mode, the follow sequence is required:	key = ON,
					//	switch = CENTER -> DOWN x 10 in 5 seconds.
					else if(	( SwitchEventStruct.SwitchPosOld == CENTER )
							 &&	( SwitchEventStruct.SwitchPosNew == DOWN)	){
							 
						//Falling edge
						if( !StateNormalDemoCount ){
						
							//Grab start time
							StateNormalDemoTime = SwitchEventStruct.SwitchTimeNew;
							
							//Increment count
							StateNormalDemoCount++;
						
						}//end !StateNormalDemoCount
						else{
						
							//Non-zero count already; so keep incrementing
							StateNormalDemoCount++;
							
							//See if we've reached the needed counts within the time limits
							if(		(StateNormalDemoCount >= DEMO_CNT_REQ )
								&&	( (SwitchEventStruct.SwitchTimeNew - StateNormalDemoTime ) <= DEMO_TIMEOUT ) ){
								
								StateNormalDemoCount = 0;
								
								CurrentState = STATE_DEMO;
								
							}//end if
						
						}//end else
							 
					}//end CENTER to DOWN
					
					//Reset Flag
					SwitchEventStruct.SwitchEventFlag = FALSE;
				
				}//end SwitchEventFlag
		
			}//end KeyPosNew == ON
			
			//Timeouts
			//Turn off interrrupts
			INTR_OFF;
			
			//Handle StateNormalLockCount timeout
			if(	(MS_TIMER - StateNormalLockTime) > LOCKED_TIMEOUT ){
			
				//We've timedout, reset count
				StateNormalLockCount = 0;
			
			}//end > LOCKED_TIMEOUT
			
			//StateNormalDemoTime timeout
			if( (MS_TIMER - StateNormalDemoTime) > DEMO_TIMEOUT ){
			
				StateNormalDemoCount = 0;
			
			}//end if
			
			//Turn interrupts back on
			INTR_ON;
			
		}//end STATE_NORMAL		
		else if(CurrentState == STATE_LOCKED){
		
			//We need to set the servo to the closed position if
			//	we're not yet initialized
			if( !StateLockedInit ){
			
				//Set the servo position
				INTR_OFF;
				DesiredDutyCycle = ServoParamsRamPtr->LowerLimit;
				INTR_ON;
				
				//Reset variables
				StateLockedEdgeCount	= 0;
				StateLockedLockTime		= 0;
				//Set the flag
				StateLockedInit 		= TRUE;
			
			}//end if
			else{
			
				//You can only exit lock mode if key is on
				if( KeyPosNew == ON ){
			
					//We've been initilized, so just look for the
					//	needed edges to exit this state
					if( SwitchEventStruct.SwitchEventFlag ){
					
						//Look for proper edges within timeout
						if(		( SwitchEventStruct.SwitchPosOld == DOWN )
							&&	( SwitchEventStruct.SwitchPosNew == CENTER) ){
							
							//We have a DOWN to CENTER rising edge
							if( !StateLockedEdgeCount ){
							
								//This is the first rising edge
								//Grab the time
								StateLockedLockTime = SwitchEventStruct.SwitchTimeNew;
								
								//Increment the count
								StateLockedEdgeCount++;
							
							}//end !EdgeCount
							else{
							
								//We've already gotten an edge
								//Increment the count
								StateLockedEdgeCount++;
								
								//See if we have enough edges before the timeout
								if(		( StateLockedEdgeCount >= LOCKED_CNT_REQ )
									&&	( ( SwitchEventStruct.SwitchTimeNew - StateLockedLockTime ) <= LOCKED_TIMEOUT ) ){
									
									//We have enough edges before the timeout
									//Reset the init flag
									StateLockedInit = FALSE;
									
									//Reset edge count
									StateLockedEdgeCount	= 0;
									
									//Set state
									CurrentState = STATE_NORMAL;
									
								}//end if
									
							}//end else
							
						}//end DOWN to CENTER
					
						//Reset Flag
						SwitchEventStruct.SwitchEventFlag = FALSE;
					
					}//end SwitchEventFlag
				
				}//end if KeyPosNew == ON;
			
			}//end else
			
			
			//Check for timeout
			INTR_OFF;
			if( (MS_TIMER - StateLockedLockTime) > LOCKED_TIMEOUT ){
			
				//Reset edge count
				StateLockedEdgeCount	= 0;
			
			}//end if
			INTR_ON;
		
		}//end STATE_LOCKED
		else if(CurrentState == STATE_DEMO ){
		
			//This is for demonstration purposes only.
			//	It will cause the servo to oscillate between
			//	the upper and lower limit.
			//Code to exit this state
			
			if(!StateDemoInit){
			
				StateDemoNormalSpeed = ServoParamsRamPtr->Speed;
				
				
				INTR_OFF;
				ServoParamsRamPtr->Speed = DEMO_SPEED;
				INTR_ON;
				
				StateDemoInit = TRUE;
			
			}//end !Init
			
			if( KeyPosNew == ON ){
			
				//Check for switch edges
				if( SwitchEventStruct.SwitchEventFlag ){
				
					if(		( SwitchEventStruct.SwitchPosOld == CENTER )
						&&	( SwitchEventStruct.SwitchPosNew == DOWN ) ) {
						
						if(!StateDemoEdgeCount){
						
							//Grab time
							StateDemoTimeout = SwitchEventStruct.SwitchTimeNew;
							
							//Increment count
							StateDemoEdgeCount++;
						
						}//end !EdgeCount
						else{
						
							//Increment count
							StateDemoEdgeCount++;
							
							if( 	( StateDemoEdgeCount >= DEMO_CNT_REQ )
								&&	( ( SwitchEventStruct.SwitchTimeNew - StateDemoTimeout ) <= DEMO_TIMEOUT) ){
								
								StateDemoEdgeCount = 0;
								
								StateDemoInit = FALSE;
								
								INTR_OFF;
								ServoParamsRamPtr->Speed = StateDemoNormalSpeed;
								INTR_ON;
								
								CurrentState = STATE_NORMAL;
								
							}//end if
						
						}//end else
						
					}//end CENTER to DOWN				
				
					//Reset flag
					SwitchEventStruct.SwitchEventFlag = FALSE;
					
				}//end SwitchEventFlag
			
			}//end if KeyPosNew == ON
			
			//Cycle from upper to lower limit and back
			if(StateDemoCycleFlag){
			
				if(DesiredDutyCycle == ServoParamsRamPtr->UpperLimit){
					INTR_OFF;
					DesiredDutyCycle = ServoParamsRamPtr->LowerLimit;
					StateDemoCycleTime = MS_TIMER;
					INTR_ON;
				}//end if
				else{
					INTR_OFF;
					DesiredDutyCycle = ServoParamsRamPtr->UpperLimit;
					StateDemoCycleTime = MS_TIMER;
					INTR_ON;
				}
				
				StateDemoCycleFlag = FALSE;
					
			}//end if

			//Handle timeouts
			//Turn off interrupts
			INTR_OFF;
			//Check for edge timeout
			if( (MS_TIMER - StateDemoTimeout) > DEMO_TIMEOUT){
			
				StateDemoEdgeCount = 0;
			
			}//end if
			
			//check for cycle timeout
			if( (MS_TIMER - StateDemoCycleTime) > DEMO_CYCLE_TIME){

				//Reset Flag
				StateDemoCycleFlag = TRUE;
			
			}//end if
			
			//Turn interrutps back on
			INTR_ON;
			
		}//end STATE_DEMO
		
		//Reset WDT
		wdt_reset();

	}//end for(;;)

	//So compiler doesn't complain
	return( 0 );

} /* end main */


//Interrupt service routine for TOC2 compare match
SIGNAL(SIG_OUTPUT_COMPARE2){
#if 0
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
	static uint8_t	SampleCount;
	static uint16_t	SpeedTimer;
	static uint16_t	HumCount;
	
	//Increment the global ms count
	MS_TIMER++;
	
	//Count to slow down the input sample rate
	if(!SampleCount){
	
		//SampleFlag has reached 0, set the signal flag
		SampleFlag 	= TRUE;
		
		//And reset the count
		SampleCount = SAMPLE_DIV;
	}
	else
		//Sample flag is still non-zero, so just decrement it
		SampleCount--;
	
	//Check to see if we are in between servo steps by testing timer count
	if(!SpeedTimer){
	
		//Limit checking of DesiredDutyCycle
		if		( DesiredDutyCycle < PWM_CLSD_LIM )
			DesiredDutyCycle = PWM_CLSD_LIM;
		else if( DesiredDutyCycle > PWM_OPEN_LIM )
			DesiredDutyCycle = PWM_OPEN_LIM;
		
	
		//Set appropriate value to move the servo at the correct rate
		if		( CurrentDutyCycle < DesiredDutyCycle - (PWM_ADJ_RESOLUTION+1) ){
			
			//Add the minimum step size to the duty cycle
			CurrentDutyCycle += PWM_ADJ_RESOLUTION;
			
			//Wait until pin is low
			while( TCNT1 <= OCR1A ){};
			
			//Write Value to Servo
			SetPWMDuty(CurrentDutyCycle);
			
			//turn on PWM
			PWM_ON;
			
			SpeedTimer = ServoParamsRamPtr->Speed;
		
		}//end if Current < Upper
		else if( CurrentDutyCycle > DesiredDutyCycle + (PWM_ADJ_RESOLUTION+1) ){
			
			//Subtract the minimum step size to the duty cycle
			CurrentDutyCycle -= PWM_ADJ_RESOLUTION;
			
			//Wait until pin is low
			while( TCNT1 <= OCR1A ){};
			
			//Write Value to Servo
			SetPWMDuty(CurrentDutyCycle);
			
			//turn on PWM
			PWM_ON;
		
			//Reset speed counter
			SpeedTimer = ServoParamsRamPtr->Speed;
			
		}//end if Current > Lower
		else{
		
			//Check to see if PWM is on and that we aren't counting yet
			if( (DDRB & (1<<PB1)) && !HumCount ){
			
				//PWM is on, we haven't started timing, so reset the counter
				HumCount = HUM_TIMEOUT;
			
			}//end if
		
		}//end else
		
	}//end if(!SpeedTimer)
	else
		SpeedTimer--;
	
	//Check hum timeout value
	if(HumCount){
	
		//If HumCount is non-zero, decrement it
		HumCount--;
	
		//If HumCount is zero now and we are in the NORMAL or LOCKED or DEMO state, turn off PWM
		if(!HumCount && ( 		( CurrentState == STATE_NORMAL ) 
							||	( CurrentState == STATE_LOCKED ) 
							||	( CurrentState == STATE_DEMO   ) ) ){
			
			//We've timed out and therefore need to shut off the PWM
			//First we need to wait until the OC2B pin is low
			while( TCNT1 <= OCR1A ){};
			
			//We've checked to make sure pin is low, turn off PWM
			PWM_OFF;
			
		}//end if
	
	}//end if
#endif
}//end SIG_OUTPUT_COMPARE0
