#ifndef HOMING_OPERATIONS_H
#define HOMING_OPERATIONS_H

#include "../core/01_CommonDefinitions.h"
#include "../hardware/05_MotorControl.h"
#include "../hardware/07_SwitchSensor.h"

// Homing operations variables
extern int homingAttemptCount;
extern bool isHomingComplete;
extern bool isCutMotorHomed;
extern bool isPositionMotorHomed;

// Initialize homing variables
void initializeHomingVariables();

// Main homing functions
bool performHomingStep(int currentSubState, int& nextSubState);
void resetHomingProgress();

// Specific homing operations
bool moveAwayFromCutMotorHome(int& nextSubState);
bool moveAwayFromPositionMotorHome(int& nextSubState);
bool homeCutMotor(int& nextSubState, ErrorType& errorType);
bool homePositionMotor(int& nextSubState, ErrorType& errorType);

// Helper functions
void moveAwayThenHomeCutMotor();
void moveAwayThenHomePositionMotor();

// Error handling function
void resetErrorAndHomeSystem();

#endif // HOMING_OPERATIONS_H 