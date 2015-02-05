/****************************************************************************
 Module
   DCService.c

 Description
   This file contains the functions that will control the DC motor.
****************************************************************************/
//#define TEST

/****************************************************************************
 Include Files
****************************************************************************/
#include "DCMotors.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "termio.h"
#include "ES_Port.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

/****************************************************************************
 Defines
****************************************************************************/
/*--- Port Pins for the DC Motor (using PWM Port B) ---*/
#define ALL_BITS (0xff <<2)
#define DC_MOTOR1  GPIO_PIN_4	// 	DC_MOTOR1		Port Pin B4		(PWM output for DC motor 1)
#define DC_MOTOR2  GPIO_PIN_6	// 	DC_MOTOR2		Port Pin B6		(PWM output for DC motor 2)

/*--- PWM Defines ---*/
// 40,000 ticks per microS assumes a 40Mhz clock, we will use SysClk/32 for PWM
#define PWMTicksPeruS 40
// set 200 Hz frequency so 5mS period
#define PeriodInuS 100
#define BitsPerNibble 4
#define PWM0 0
#define PWM1 1

/*--- Other Defines ---*/
#define clrScrn() 	printf("\x1b[2J")

/****************************************************************************
 Module Variables
****************************************************************************/
static uint8_t PWM0DutyCycle = 50;
static uint8_t PWM1DutyCycle = 50;

/****************************************************************************
 Private Function Prototypes
****************************************************************************/
void SetPWMDuty(uint8_t DutyCycle, uint8_t PWM);

/****************************************************************************
 Function Implementations
****************************************************************************/

/****************************************************************************
 Function
     InitMotors
 Parameters
     void
 Returns
     void
 Description
     Initializes the PWM pins for the DC motors on Port B
****************************************************************************/
void InitMotors(void)
{
	// Initialize the PWM
  volatile uint32_t Dummy; // use volatile to avoid over-optimization
	// start by enabling the clock to the PWM Module (PWM0 and PWM1)
  HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
	
	// enable the clock to Port B
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
	
	// Select the PWM clock as System Clock/32
  HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) |
    (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
	// make sure that the PWM module clock has gotten going
	while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0 )
    ;
	
	// disable the PWM while initializing
  HWREG( PWM0_BASE+PWM_O_0_CTL ) = 0;
  HWREG( PWM0_BASE+PWM_O_1_CTL ) = 0;
	
	// program generator A to go to 0 at rising comare A, 1 on falling compare A  
  HWREG( PWM0_BASE+PWM_O_0_GENA) = 
    (PWM_0_GENA_ACTCMPAU_ZERO | PWM_0_GENA_ACTCMPAD_ONE );
  HWREG( PWM0_BASE+PWM_O_1_GENA) = 
    (PWM_1_GENA_ACTCMPAU_ZERO | PWM_1_GENA_ACTCMPAD_ONE );
	
	// Set the PWM period. Since we are counting both up & down, we initialize
	// the load register to 1/2 the desired total period
  HWREG( PWM0_BASE+PWM_O_0_LOAD) = (PeriodInuS * PWMTicksPeruS)>>1;
  HWREG( PWM0_BASE+PWM_O_1_LOAD) = (PeriodInuS * PWMTicksPeruS)>>1;
	
	// Set the initial Duty cycle on A to 50% by programming the compare value
	// to 1/2 the period to count up (or down) 
  HWREG( PWM0_BASE+PWM_O_0_CMPA) = ((PeriodInuS * PWMTicksPeruS)-1)>>2;
  HWREG( PWM0_BASE+PWM_O_1_CMPA) = ((PeriodInuS * PWMTicksPeruS)-1)>>2;
  
	// set changes to the PWM output Enables to be locally syncronized to a 
	// zero count
  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
      ~(PWM_ENUPD_ENUPD0_M | PWM_ENUPD_ENUPD1_M)) |
      (PWM_ENUPD_ENUPD0_LSYNC | PWM_ENUPD_ENUPD1_LSYNC);
//  HWREG(PWM0_BASE+PWM_O_ENUPD) =  (HWREG(PWM0_BASE+PWM_O_ENUPD) & 
//      ~(PWM_ENUPD_ENUPD0_M | PWM_ENUPD_ENUPD1_M)) |
//      (PWM_ENUPD_ENUPD0_LSYNC | PWM_ENUPD_ENUPD1_LSYNC);

	// enable the PWM outputs
  HWREG( PWM0_BASE+PWM_O_ENABLE) |= (PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM0EN);
//  HWREG( PWM1_BASE+PWM_O_ENABLE) |= (PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM0EN);

	// now configure the Port B pins to be PWM outputs
	// start by selecting the alternate function for DC_MOTOR1 (PB4) and DC_MOTOR2 (PB6)
  HWREG(GPIO_PORTB_BASE+GPIO_O_AFSEL) |= (BIT4HI | BIT6HI);
  
// now choose to map PWM to those pins, this is a mux value of 4 that we
// want to use for specifying the function on bit 4 and bit 6
  HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) & 0x00fffff) + (4<<(4*BitsPerNibble));
  HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) = 
    (HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) & 0x00fffff) + (4<<(6*BitsPerNibble));
	
// Enable pin 4 and 6 on Port B for digital I/O
	HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (BIT4HI | BIT6HI);
// make pin 4 and 6 on Port B into outputs
	HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (BIT4HI | BIT6HI);
// set the up/down count mode and enable the PWM generator
  HWREG(PWM0_BASE+ PWM_O_0_CTL) |= (PWM_0_CTL_MODE | PWM_0_CTL_ENABLE);
  HWREG(PWM0_BASE+ PWM_O_1_CTL) |= (PWM_1_CTL_MODE | PWM_1_CTL_ENABLE);
 
	printf("DC Motors Initialized.\r\n");
}

/****************************************************************************
 Function
     SetPWMDuty
 Parameters
     uint8_t DutyCycle
     uint8_t PWM (#defines are PWM0 or PWM1)
 Returns
     void
 Description
     Sets the PWM duty cycle for the DC motor
****************************************************************************/
void SetPWMDuty(uint8_t DutyCycle, uint8_t PWM) {
  switch (PWM) {
    case PWM0:
      PWM0DutyCycle = DutyCycle;
      if (PWM0DutyCycle == 0) {
      // Set the Duty cycle on A to 0% by programming the compare value
      // to 0. However, since the CmpADn action (set to one) wins, we also
      // need to disable the output  
        HWREG( PWM0_BASE+PWM_O_0_CMPA) = 0;
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM0EN;
      } else if (PWM0DutyCycle == 100) {
        // Set the Duty cycle on A to 100% by programming the compare value
        // to the load value. Since the CmpBDn action (set to one) wins, we get 100%
        HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM0EN;
        HWREG( PWM0_BASE+PWM_O_0_CMPA) = HWREG( PWM0_BASE+PWM_O_0_LOAD);
      } else {
        HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM0EN;
        uint32_t DutyCycleTicks = (PeriodInuS * PWMTicksPeruS) / 2 * PWM0DutyCycle / 100;
        HWREG( PWM0_BASE+PWM_O_0_CMPA) = DutyCycleTicks;
      }
      break;
    case PWM1:
      PWM1DutyCycle = DutyCycle;
      if (PWM1DutyCycle == 0) {
      // Set the Duty cycle on A to 0% by programming the compare value
      // to 0. However, since the CmpADn action (set to one) wins, we also
      // need to disable the output  
        HWREG( PWM0_BASE+PWM_O_1_CMPA) = 0;
        HWREG( PWM0_BASE+PWM_O_ENABLE) &= ~PWM_ENABLE_PWM2EN;
      } else if (PWM1DutyCycle == 100) {
        // Set the Duty cycle on A to 100% by programming the compare value
        // to the load value. Since the CmpBDn action (set to one) wins, we get 100%
        HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM2EN;
        HWREG( PWM0_BASE+PWM_O_1_CMPA) = HWREG( PWM0_BASE+PWM_O_1_LOAD);
      } else {
        HWREG( PWM0_BASE+PWM_O_ENABLE) |= PWM_ENABLE_PWM2EN;
        uint32_t DutyCycleTicks = (PeriodInuS * PWMTicksPeruS) / 2 * PWM1DutyCycle / 100;
        HWREG( PWM0_BASE+PWM_O_1_CMPA) = DutyCycleTicks;
      }
      break;
    default:
      break;
  }
}

/****************************************************************************
 Function
     RunMotors
 Parameters
     uint8_t DriveMode
 Returns
     void
 Description
     Main function to run the motors. The drive modes are:
        #define STOP 0    //this disables PWM to each motor
        #define CW 1      //this drives the motors to rotate the robot in the clockwise direction
        #define CCW 2     //this drives the motors to rotate the robot in the counterclockwise direction
        #define FHALF 3   //this drives the motors to move the robot forward at half speed
        #define FFULL 4   //this drives the motors to move the robot forward at full speed
        #define RHALF 5   //this drives the motors to move the robot backward at half speed
        #define RFULL 6   //this drives the motors to move the robot backward at full speed
****************************************************************************/
void RunMotors(uint8_t DriveMode) {
  switch (DriveMode) {
    case STOP:
      SetPWMDuty(50, PWM0);
      SetPWMDuty(50, PWM1);
      break;
    case CCW:
      SetPWMDuty(100, PWM0);
      SetPWMDuty(0, PWM1);
      break;
    case CW:
      SetPWMDuty(0, PWM0);
      SetPWMDuty(100, PWM1);
      break;
    case RHALF:
      SetPWMDuty(75, PWM0);
      SetPWMDuty(75, PWM1);
      break;
    case RFULL:
      SetPWMDuty(100, PWM0);
      SetPWMDuty(100, PWM1);
      break;
    case FHALF:
      SetPWMDuty(25, PWM0);
      SetPWMDuty(25, PWM1);
      break;
    case FFULL:
      SetPWMDuty(0, PWM0);
      SetPWMDuty(0, PWM1);
      break;
    default:
      break;
  }
}


#ifdef TEST 
/* Test Harness for the DC motors */ 
#include "termio.h" 
int main(void) 
{ 
	// Set the clock to run at 40MhZ using the PLL and 16MHz external crystal
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
			| SYSCTL_XTAL_16MHZ);
	TERMIO_Init(); 
	clrScrn();
	printf("In test harness of DC motors\r\n");
	InitMotors();
	RunMotors(CW);
	return 0;
}
#endif

