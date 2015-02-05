/****************************************************************************
 Module
   TapeSensor.h

 Description
   Contains the the function that will read the tape sensor output and
   report its state
****************************************************************************/

#ifndef TAPESENSOR_H
#define TAPESENSOR_H

#include <stdbool.h>

// Initializes the tape sensor
void InitTapeSensor(void);

// Checks if the tape sensor is covered
// HIGH = covered
// LOW = not covered
bool TapeSensorCovered(void);

#endif
