/****************************************************************************
 Module
   TapeSensor.c

 Description
   Contains the the function that will read the tape sensor output and
   report its state
****************************************************************************/

 //#define TEST


/***********
  Includes
 ***********/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "termio.h"
#include "ES_Port.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

#include "TapeSensor.h"
#include "DCMotors.h"

/*****************
  Module Defines
 *****************/
#define ALL_BITS (0xff <<2)
#define TAPE_PORT_DEC SYSCTL_RCGCGPIO_R2 //port C
#define TAPE_PORT GPIO_PORTC_BASE // port C base
#define TAPE_PIN GPIO_PIN_7 // pin 7
#define TAPE_HI BIT7HI


/***************************
  Function Implementations
 ***************************/

// Initializes the tape sensors
void InitTapeSensor(void) {
	HWREG(SYSCTL_RCGCGPIO) |= (TAPE_PORT_DEC);
	HWREG(TAPE_PORT+GPIO_O_DEN) |= TAPE_PIN;
	HWREG(TAPE_PORT+GPIO_O_DIR) &= ~TAPE_PIN;
}

// Checks if the tape sensor is covered
// HIGH = covered
// LOW = not covered
bool TapeSensorCovered(void) {
	return (HWREG(TAPE_PORT+(GPIO_O_DATA + ALL_BITS)) & TAPE_PIN);
}


#ifdef TEST 
/* Test Harness for Tape Sensors module */ 
int main(void) 
{ 
	TERMIO_Init(); 
	printf("\n\rIn Test Harness for Tape Sensors\r\n");
	InitMotors();
	RunMotors(STOP);
	InitTapeSensor();
	while(true){
		if (TapeSensorCovered()) {
			printf("Tape sensor is covered.\r\n");
		} else {
			printf("Tape sensor is not covered.\r\n");
		}
	}
}
#endif
