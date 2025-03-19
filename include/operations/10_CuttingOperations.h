#ifndef CUTTING_OPERATIONS_H
#define CUTTING_OPERATIONS_H

#include "../core/01_CommonDefinitions.h"
#include "../hardware/05_MotorControl.h"
#include "../hardware/07_SwitchSensor.h"
#include "../hardware/08_PeripheralControl.h"

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