/****************************************************************************
 Module: Robot.c
 Description: Robot state machine which does killer stuff. 
 Author: Team crazy cool cat attacks 2/3/2015
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "bitdefs.h"

#include <stdio.h>


#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"	// Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"

#include "SSI.h"
#include "Robot.h"
#include "DCMotors.h"
#include "TapeSensor.h"
#include "BeaconSensing.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.000mS/tick timing
#define CW90_TIME 1000
#define CW45_TIME 500
#define CCW90_TIME 1000
#define CCW45_TIME 500
#define SPI_TIME 10 //Querries command generator every 10ms

//Robot States
#define NORMAL 0
#define LOOKINGFORIR 1
#define LOOKINGFORTAPE 2

//#define TEST

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for ovehead
static ES_Event DeferralQueue[3+1];
static uint8_t RobotState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
Function: InitRobot
Parameters: uint8_t Priority
Returns: bool
Description: Initializes the Robot state machine
****************************************************************************/
bool InitRobot ( uint8_t Priority )
{
  ES_Event ThisEvent;  
  MyPriority = Priority;
	// initialize deferral queue for testing Deferal function
  ES_InitDeferralQueueWith( DeferralQueue, ARRAY_SIZE(DeferralQueue) );
	
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}

/****************************************************************************
Function: PostRobot
Parameters: ES_Event ThisEvent
Returns: bool	
Description: Posts an event to the Robot state machine
****************************************************************************/
bool PostRobot( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}
/****************************************************************************
Function: CheckRobotEvents
Parameters: void
Returns: void
Description: Bogus filler function to make the ES framework happy
****************************************************************************/
bool CheckRobotEvents(void)
{
  return false;
}

/****************************************************************************
Function: CheckTapeSensor
Parameters: void
Returns: bool (true if tape sensor has changed states)
Description: Checks to see if the tape sensor has changed states.
	If the tape sensor becomes covered, post an ES_TAPEDETECTED event
****************************************************************************/
bool CheckTapeSensor(void) {
	static bool LastState = false;
	bool ReturnVal = false;
	if (!LastState && TapeSensorCovered()) {
		//printf("Tape sensor is covered.\r\n");
		LastState = true;
		ES_Event ThisEvent;
		ThisEvent.EventType = ES_TAPEDETECTED;
		PostRobot(ThisEvent);
		ReturnVal = true;
	} else if (LastState && !TapeSensorCovered()) {
		//printf("Tape sensor is not covered.\r\n");
		LastState = false;
		ReturnVal = true;
	}
	return ReturnVal;
}

/****************************************************************************
Function: CheckIRSensor
Parameters: void
Returns: bool (true if IR sensor has changed states)
Description: Checks to see if the IR sensor has changed states.
	If the IR sensor detects a signal, post an ES_IRDETECTED event
****************************************************************************/
bool CheckIRSensor(void) {
	static bool LastState = false;
	bool ReturnVal = false;
	if (!LastState && IsBeaconSensed()) {
		//printf("IR sensor detected a signal.\r\n");
		LastState = true;
		ES_Event ThisEvent;
		ThisEvent.EventType = ES_IRDETECTED;
		PostRobot(ThisEvent);
		ReturnVal = true;
	} else if (LastState && !IsBeaconSensed()) {
		//printf("IR sensor lost the signal.\r\n");
		LastState = false;
		ReturnVal = true;
	}
	return ReturnVal;
}


/****************************************************************************
Function: RunRobot
Parameters: ES_Event ThisEvent
Returns: ES_Event
Description: Run function of the robot state machine
****************************************************************************/
ES_Event RunRobot( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType){
    case ES_INIT :
      printf("\rES_INIT received in Service %d\r\n", MyPriority);		
			InitSSI(); //Initialize Tiva as master on SSI0
			InitMotors();
			InitTapeSensor();
			InitInputCapturePeriod();
			ES_Timer_InitTimer(SPI_TIMER, SPI_TIME); 
			RobotState = NORMAL;
      break;
    case ES_TIMEOUT :
			if (ThisEvent.EventParam == 1){ //Time to querry the command generator
				ES_Timer_InitTimer(SPI_TIMER, SPI_TIME); //Reinitialize SPI timer
				QuerryCommandGenerator();
			}else if(ThisEvent.EventParam == 0){
				RunMotors(STOP);
				printf("\rRotation stopped after TIMEOUT\n");
				RobotState = NORMAL;
			}
      break;
		case ES_IRDETECTED :
			if (RobotState == LOOKINGFORIR){
				RunMotors(STOP);
				RobotState = NORMAL;
				printf("\rIR found, stopping motor\n");
			}
      break;
		case ES_TAPEDETECTED :
			if (RobotState == LOOKINGFORTAPE){
				RunMotors(STOP);
				RobotState = NORMAL;
				printf("\rTape found, stopping motor\n");
			}
      break;
		case ES_NEWCOMMAND :
			switch (ThisEvent.EventParam){
				case 0x00:
					printf("\rSTOP\n");
					RunMotors(STOP);
					RobotState = NORMAL;
					break;
				case 0x02:
					printf("\rRCW90\n");
					RunMotors(CW);
					ES_Timer_InitTimer(ROTATION_TIMER, CW90_TIME);
					RobotState = NORMAL;
					break;
				case 0x03:
					printf("\rRCW45\n");
					RunMotors(CW);
					ES_Timer_InitTimer(ROTATION_TIMER, CW45_TIME);
					RobotState = NORMAL;
					break;
				case 0x04:
					printf("\rRCCW90\n");
					RunMotors(CCW);
					ES_Timer_InitTimer(ROTATION_TIMER, CCW90_TIME);
					RobotState = NORMAL;
					break;
				case 0x05:
					printf("\rRCCW45\n");
					RunMotors(CCW);
					ES_Timer_InitTimer(ROTATION_TIMER, CCW45_TIME);
					RobotState = NORMAL;
					break;
				case 0x08:
					printf("\rDF50\n");
					RunMotors(FHALF);
					RobotState = NORMAL;
					break;
				case 0x09:
					printf("\rDF100\n");
					RunMotors(FFULL);
					RobotState = NORMAL;
					break;
				case 0x10:
					printf("\rDR50\n");
					RunMotors(RHALF);
					RobotState = NORMAL;
					break;
				case 0x11:
					printf("\rDR100\n");
					RunMotors(RFULL);
					RobotState = NORMAL;
					break;
				case 0x20:
					printf("\rALIGNING_IR\n");
					RunMotors(CW);
					RobotState = LOOKINGFORIR;
					break;
				case 0x40:
					printf("\rALIGNING_TAPE\n");
				  RunMotors(FHALF);
				  RobotState = LOOKINGFORTAPE;
					break;
			}
		default:			
      break;
	}
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
#ifdef TEST
#include "termio.h"
int main(void){
	TERMIO_Init();
	InitSSI();
	printf("\rIn test harness for Robot.c. SSI just initiated\r\n");
	/*
	ES_Event ThisEvent;
	ThisEvent.EventType = ES_NEWCOMMAND;
	ThisEvent.EventParam = 0x02; 
	*/
	
	
	/*
	//Querry the CG 8 times in 1 second intervals
	for (int i = 0; i < 8; i++){
		HWREG(SSI0_BASE + SSI_O_DR) = 0xAA; //Query the command generator
	
		for (int j = 0; j < 1000; j++){ // Delay for aprox. 1 second
			volatile uint32_t Dummy;
			Dummy = HWREG(SYSCTL_RCGCGPIO); 
		}
	}
	
	//Read the RX FIFO and print out data
	for (int k = 0; k < 8; k++){		
		uint16_t Data = HWREG(SSI0_BASE + SSI_O_DR);
		printf("\rReceived instruction: %#04x\n", Data);
	} 
	*/
	for(;;)
	;
}
#endif
/*------------------------------ End of file ------------------------------*/

