#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "../core/01_CommonDefinitions.h"
#include "../hardware/06_LEDControl.h"
#include "../hardware/07_SwitchSensor.h"
#include "../hardware/05_MotorControl.h"
#include "../hardware/08_PeripheralControl.h"
#include "10_HomingOperations.h"
#include "11_CuttingOperations.h"
#include "12_YesWoodOperations.h"
#include "13_NoWoodOperations.h"

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
void handleCuttingState();
void handleYesWoodState();
void handleNoWoodState();
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