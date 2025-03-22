#ifndef HOMING_H
#define HOMING_H

#include "../core/01_CommonDefinitions.h"
#include "../core/04_Config.h"

// Function to handle the homing state
void handleHomingState();

// Initialize variables needed for homing
void initializeHomingVariables();

// Perform a single step of the homing process
// Returns true if the step is complete
// nextSubState will be set to the next sub-state if it changes
bool performHomingStep(int currentSubState, int &nextSubState);

// Sub-state handlers for the homing process
bool homeCutMotor(int &nextSubState);
bool homePositionMotor(int &nextSubState);
bool finalizeHoming(int &nextSubState);

#endif // HOMING_H 