#ifndef YES_WOOD_H
#define YES_WOOD_H

#include "../core/01_CommonDefinitions.h"
#include "../hardware/06_LEDControl.h"
#include "../hardware/07_SwitchSensor.h"
#include "../hardware/08_PeripheralControl.h"
#include "../hardware/05_MotorControl.h"
#include "09_StateMachine.h"

// External references to shared variables
extern bool hasSuctionBeenChecked;
extern bool hasTransferArmBeenSignaled;

// Function prototypes for yes wood operations
void handleYesWoodState();
void waitForCycleSwitch();
void showYesWoodIndicator();

#endif // YES_WOOD_H 