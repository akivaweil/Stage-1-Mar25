#ifndef YES_WOOD_OPERATIONS_H
#define YES_WOOD_OPERATIONS_H

#include "CommonDefinitions.h"
#include "LEDControl.h"
#include "SwitchSensor.h"
#include "PeripheralControl.h"
#include "StateMachine.h"

// Function prototypes for yes wood operations
void handleYesWoodState();
void waitForCycleSwitch();
void showYesWoodIndicator();

#endif // YES_WOOD_OPERATIONS_H 