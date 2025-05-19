#include "../../include/operations/10_Homing.h"
#include "../../include/hardware/05_MotorControl.h"
#include "../../include/hardware/07_SwitchSensor.h"
#include "../../include/hardware/06_LEDControl.h"
#include "../../include/operations/09_StateMachine.h"

// Homing state variables
int homingAttemptCount = 0;
bool isHomingComplete = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;
unsigned long homingStartTime = 0;

// Initialize homing variables
void initializeHomingVariables() {
    homingAttemptCount = 0;
    isHomingComplete = false;
    isCutMotorHomed = false;
    isPositionMotorHomed = false;
    homingStartTime = millis();
}

// The main handler for the homing state
void handleHomingState() {
    // Call the state machine to process this state
    int nextSubState = subState;
    
    // Call the homing operations module to perform the current homing step
    bool stepComplete = performHomingStep(subState, nextSubState);
    
    // Update substate if changed
    if (nextSubState != subState) {
        subState = nextSubState;
    }
    
    // Check for completion or errors
    if (stepComplete) {
        if (isHomingComplete) {
            enterState(READY_STATE);
        } else if (currentError == CUT_MOTOR_HOME_ERROR) {
            enterState(CUT_MOTOR_HOME_ERROR_STATE);
        } else if (currentError == POSITION_MOTOR_HOME_ERROR) {
            enterState(POSITION_MOTOR_HOME_ERROR_STATE);
        }
    }
}

// Perform a single step of the homing process
bool performHomingStep(int currentSubState, int &nextSubState) {
    // Blink blue LED during homing
    blinkBlueLed();
    
    // Check for homing timeout
    if (millis() - homingStartTime > HOMING_TIMEOUT * 2) {
        // If we couldn't home either motor, signal error state
        if (!isCutMotorHomed) {
            currentError = CUT_MOTOR_HOME_ERROR;
            return true;
        } else if (!isPositionMotorHomed) {
            currentError = POSITION_MOTOR_HOME_ERROR;
            return true;
        }
    }
    
    // Handle the current sub-state
    switch (currentSubState) {
        case 0: // Home cut motor
            return homeCutMotor(nextSubState);
            
        case 1: // Home position motor
            return homePositionMotor(nextSubState);
            
        case 2: // Finalize homing
            return finalizeHoming(nextSubState);
            
        default:
            nextSubState = 0; // Reset to first sub-state if invalid
            return false;
    }
}

// Home the cut motor (sub-state 0)
bool homeCutMotor(int &nextSubState) {
    // If cut motor is already homed, move to next sub-state
    if (isCutMotorHomed) {
        nextSubState = 1;
        return false;
    }
    
    // Check if cut motor has reached home position
    if (readCutMotorHomingSwitch()) {
        stopCutMotor();
        isCutMotorHomed = true;
        nextSubState = 1; // Move to position motor homing
        return false;
    }
    
    // Move motor toward home position
    moveCutMotorToHome();
    
    // Still running this sub-state
    return false;
}

// Home the position motor (sub-state 1)
bool homePositionMotor(int &nextSubState) {
    // If position motor is already homed, move to next sub-state
    if (isPositionMotorHomed) {
        nextSubState = 2;
        return false;
    }
    
    // Check if position motor has reached home position
    if (readPositionMotorHomingSwitch()) {
        stopPositionMotor();
        isPositionMotorHomed = true;
        nextSubState = 2; // Move to finalizing homing
        return false;
    }
    
    // Move motor toward home position
    movePositionMotorToHome();
    
    // Still running this sub-state
    return false;
}

// Finalize homing (sub-state 2)
bool finalizeHoming(int &nextSubState) {
    // Ensure both motors are homed
    if (isCutMotorHomed && isPositionMotorHomed) {
        // Set completion flag
        isHomingComplete = true;
        
        // Reset positions in the motor control
        resetCutMotorPosition();
        resetPositionMotorPosition();
        
        // Sub-state complete
        return true;
    }
    
    // If motors aren't homed yet, go back to appropriate sub-state
    if (!isCutMotorHomed) {
        nextSubState = 0;
    } else if (!isPositionMotorHomed) {
        nextSubState = 1;
    }
    
    return false;
} 