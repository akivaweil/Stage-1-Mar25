#ifndef CUTTING_OPERATIONS_H
#define CUTTING_OPERATIONS_H

#include "CommonDefinitions.h"
#include "MotorControl.h"
#include "SwitchSensor.h"
#include "PeripheralControl.h"

// Constants for cutting operations
extern const float CUT_MOTOR_CUTTING_SPEED;
extern const float WAS_WOOD_SUCTIONED_POSITION;
extern const float TRANSFER_ARM_SIGNAL_POSITION;

// Function prototypes for cutting operations
void handleCuttingState();
bool checkSuctionAndHandleErrors();
void moveToCuttingPosition();
void performCut();
void returnToHomeAfterCut();
bool isCutComplete();
void transitionToWoodDetectionState();

// External declarations for shared variables
extern bool hasSuctionBeenChecked;
extern bool hasTransferArmBeenSignaled;

#endif // CUTTING_OPERATIONS_H 