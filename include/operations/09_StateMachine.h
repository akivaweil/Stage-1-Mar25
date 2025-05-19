#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// Include core definitions
#include "../core/01_CommonDefinitions.h"
#include "../core/02_PinDefinitions.h"
#include "../core/03_Utilities.h"

// Include modular state handlers
#include "10_Homing.h"
#include "11_Cutting.h"
#include "12_YesWood.h"
#include "13_NoWood.h"
#include "15_Error.h"

// External variables declarations
extern State currentState;
extern ErrorType currentError;
extern int subState;
extern unsigned long stateStartTime;
extern unsigned long subStateTimer;
extern bool hasSuctionBeenChecked;
extern bool hasTransferArmBeenSignaled;
extern bool needCycleSwitchToggle;
extern int homingAttemptCount;

// State machine functions
void updateStateMachine();
void enterState(State newState);
void resetErrorAndHomeSystem();
void moveAwayThenHomeCutMotor();
void moveAwayThenHomePositionMotor();
void ensureMotorsAtHome();

// State handler functions
void handleStartupState();
// handleHomingState() now defined in 10_Homing.h
void handleReadyState();
void handleReloadState();
// executeCutting() defined in 11_Cutting.h
// handleYesWoodState() defined in 12_YesWood.h 
// handleNoWoodState() defined in 13_NoWood.h
void handleErrorState();
void handleWoodSuctionErrorState();
void handleCutMotorHomeErrorState();
void handlePositionMotorHomeErrorState();

#endif // STATE_MACHINE_H 