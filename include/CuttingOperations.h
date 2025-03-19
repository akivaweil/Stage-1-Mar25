#ifndef CUTTING_OPERATIONS_H
#define CUTTING_OPERATIONS_H

#include "CommonDefinitions.h"

// Constants for cutting operations
extern const float CUT_MOTOR_CUTTING_SPEED;
extern const float WAS_WOOD_SUCTIONED_POSITION;
extern const float TRANSFER_ARM_SIGNAL_POSITION;

// Function declarations
void CutMotor_CUTTING_settings();
void handleCuttingState();

#endif // CUTTING_OPERATIONS_H 