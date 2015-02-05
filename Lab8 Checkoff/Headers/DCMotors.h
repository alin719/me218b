/****************************************************************************
 Module
   DCMotors.h

 Description
   This file contains the functions that will control the DC motor.
****************************************************************************/

#ifndef DCMOTORS_H
#define DCMOTORS_H

/**************************
  Includes
***************************/
#include <stdint.h>

/**************************
  Defines (Motor Drive Modes)
***************************/
#define STOP 0    //this disables PWM to each motor
#define CW 1      //this drives the motors to rotate the robot in the clockwise direction
#define CCW 2     //this drives the motors to rotate the robot in the counterclockwise direction
#define FHALF 3   //this drives the motors to move the robot forward at half speed
#define FFULL 4   //this drives the motors to move the robot forward at full speed
#define RHALF 5   //this drives the motors to move the robot backward at half speed
#define RFULL 6   //this drives the motors to move the robot backward at full speed

/**************************
Public Function Prototypes
***************************/
// Initializes the PWM pins for the motors on Port B
// Motor 1 is on PB4, Motor 2 is on PB6
void InitMotors(void);

// Main function to run the motors
void RunMotors(uint8_t DriveMode);

#endif

