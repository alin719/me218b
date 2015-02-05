#ifndef Robot_H
#define Robot_H

#include "ES_Configure.h"
#include "ES_Types.h"

// Public Function Prototypes

bool InitRobot ( uint8_t Priority );
bool PostRobot( ES_Event ThisEvent );
bool CheckRobotEvents(void);
bool CheckTapeSensor(void);
bool CheckIRSensor(void);
ES_Event RunRobot( ES_Event ThisEvent );


#endif /* ServTemplate_H */
