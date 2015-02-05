
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "bitdefs.h"
#include "driverlib/gpio.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"

#include "SSI.h"
#include "Robot.h"


// 40,000 ticks per mS assumes a 40Mhz clock
//#define TicksPerMS 40000

static uint8_t LastCommand = 0xFF;  

void InitSSI( void ){
	// Enable clock to GPIO Port A
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
	
	// Enable clock to SSI Module 0
	HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCSSI_R0;
	
	// Wait for the GPIO Port to be ready
	while((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R0) != SYSCTL_PRGPIO_R0)
		;
		
	// Program the GPIO to use the alternate functions on the SSI pins (p650)
	HWREG(GPIO_PORTA_BASE+GPIO_O_AFSEL) |= (BIT2HI | BIT3HI | BIT4HI | BIT5HI);
	
	// Select the SSI alternate functions on those pins
	HWREG(GPIO_PORTA_BASE+GPIO_O_PCTL) = 
		(HWREG(GPIO_PORTA_BASE+GPIO_O_PCTL) & 0xff0000ff) + (2<<8) + (2<<12) + (2<<16) + (2<<20);
	
	// Program the port lines for digital I/O
	HWREG(GPIO_PORTA_BASE+GPIO_O_DEN) |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
	
	// Program the required data directions on the port lines
	HWREG(GPIO_PORTA_BASE+GPIO_O_DIR) |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);
	HWREG(GPIO_PORTA_BASE+GPIO_O_DIR) &= ~(GPIO_PIN_4);
	
	// Program the pull-up on the clock line
	HWREG(GPIO_PORTA_BASE+GPIO_O_PUR) |= GPIO_PIN_2;
	
	// Wait for the SSI0 to be ready
	while((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R0) != SYSCTL_PRSSI_R0)
		;
	
	// Make sure the SSI is disabled before programming mode bits
	HWREG(SSI0_BASE + SSI_O_CR1) &= ~(SSI_CR1_SSE);
	
	// Select master mode (MS) 
	HWREG(SSI0_BASE + SSI_O_CR1) &= ~(SSI_CR1_MS);
	
	// Select TXRIS indicatig EOT
	HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
	
	// Configure the SSI clock source to the system clock
	HWREG(SSI0_BASE + SSI_O_CC) |= SSI_CC_CS_SYSPLL;
	
	// Configure the clock pre-scaler
	HWREG(SSI0_BASE + SSI_O_CPSR) |= 0x04;
	
	// Configure the clock t (SCR), phase & polarity (SPH, SPO), mode (FRF), data size (DSS)
	HWREG(SSI0_BASE + SSI_O_CR0) |= 9<<8;
	HWREG(SSI0_BASE + SSI_O_CR0) |= (SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8);
	
	// Locally enable interrupts on TXRIS
	HWREG(SSI0_BASE + SSI_O_IM) |= SSI_IM_TXIM;
	
	// Make sure the SSI is enabled for operation
	HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_SSE;

// enable the SSI0 interrupt in the NVIC
// it is interrupt number 7 so appears in EN0 at bit 7
  HWREG(NVIC_EN0) |= BIT7HI;

// make sure interrupts are enabled globally
  __enable_irq();

}

void SSIResponse( void ){
	uint8_t NewCommand = HWREG(SSI0_BASE + SSI_O_DR);
	//printf("\rReceived instruction: %#04X\r\n", NewCommand);
	
	if (NewCommand == LastCommand || NewCommand == 0xFF) {
		return;
	}else{
		ES_Event ThisEvent;
		ThisEvent.EventType = ES_NEWCOMMAND;
		ThisEvent.EventParam = NewCommand; 
		PostRobot(ThisEvent);
		LastCommand = NewCommand; 
	}
	return;
}

void QuerryCommandGenerator (void){
	HWREG(SSI0_BASE + SSI_O_DR) = 0xAA; //Query the command generator
	return;
}


