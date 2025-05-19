#ifndef ERROR_H
#define ERROR_H

#include "../core/01_CommonDefinitions.h"
#include "../hardware/06_LEDControl.h"
#include "../hardware/07_SwitchSensor.h"
#include "../hardware/08_PeripheralControl.h"
#include "../hardware/05_MotorControl.h"

// Function prototypes
void updateErrorLED(ErrorType errorType);
void resetErrorState();
void setPositionMotorHomeErrorPattern();

#endif // ERROR_H 