#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "CommonDefinitions.h"
#include "LEDControl.h"
#include "SwitchSensor.h"
#include "MotorControl.h"
#include "PeripheralControl.h"
#include "CuttingOperations.h"
#include "YesWoodOperations.h"
#include "NoWoodOperations.h"

// State machine declarations
extern unsigned long stateStartTime;
extern unsigned long subStateTimer;
extern int homingAttemptCount;
extern bool isHomingComplete;
extern bool isCutMotorHomed;
extern bool isPositionMotorHomed;

// Error handling function
void resetErrorAndHomeSystem();

// State handlers
void handleStartupState();
void handleHomingState();
void handleReadyState();
void handleReloadState();
void handleErrorState();
void handleWoodSuctionErrorState();
void handleCutMotorHomeErrorState();
void handlePositionMotorHomeErrorState();

// State machine functions
void enterState(State newState);
void updateStateMachine();

// Helpers for recovery/error handling
void moveAwayThenHomeCutMotor();
void moveAwayThenHomePositionMotor();

#endif // STATE_MACHINE_H 