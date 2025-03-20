#include "../../include/operations/10_HomingOperations.h"

// Homing operations variables
int homingAttemptCount = 0;
bool isHomingComplete = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;

// Initialize homing variables
void initializeHomingVariables() {
  homingAttemptCount = 0;
  isHomingComplete = false;
  isCutMotorHomed = false;
  isPositionMotorHomed = false;
}

// Main function to perform a single homing step based on the current substate
bool performHomingStep(int currentSubState, int& nextSubState) {
  ErrorType errorType = NO_ERROR;
  bool stateComplete = false;
  
  switch (currentSubState) {
    case 0:  // Check if motors are already at home
      if (readCutMotorHomingSwitch()) {
        nextSubState = 1;  // Need to move cut motor away from home
      } else {
        nextSubState = 2;  // Cut motor already away from home, check position motor
      }
      
      if (readPositionMotorHomingSwitch()) {
        if (nextSubState == 2) {
          nextSubState = 3;  // Need to move position motor away from home
        }
      } else {
        if (nextSubState == 2) {
          nextSubState = 4;  // Both motors away from home, proceed to homing cut motor
        }
      }
      break;
      
    case 1:  // Move cut motor away from home switch
      stateComplete = moveAwayFromCutMotorHome(nextSubState);
      break;
      
    case 2:  // Intermediate case - should never reach here
      nextSubState = (readPositionMotorHomingSwitch()) ? 3 : 4;
      break;
      
    case 3:  // Move position motor away from home switch
      stateComplete = moveAwayFromPositionMotorHome(nextSubState);
      break;
      
    case 4:  // Home cut motor
      stateComplete = homeCutMotor(nextSubState, errorType);
      break;
      
    case 5:  // Home position motor
      stateComplete = homePositionMotor(nextSubState, errorType);
      if (stateComplete && errorType == NO_ERROR) {
        isHomingComplete = true;
      }
      break;
  }
  
  return stateComplete;
}

// Reset homing progress
void resetHomingProgress() {
  homingAttemptCount = 0;
  isHomingComplete = false;
  isCutMotorHomed = false;
  isPositionMotorHomed = false;
}

// Move cut motor away from home switch
bool moveAwayFromCutMotorHome(int& nextSubState) {
  CutMotor_HOMING_settings();
  
  if (!cutMotor.isRunning()) {
    cutMotor.move(-300);  // Move 300 steps away
  }
  
  if (!readCutMotorHomingSwitch() || cutMotor.distanceToGo() == 0) {
    cutMotor.stop();
    
    if (readPositionMotorHomingSwitch()) {
      nextSubState = 3;
    } else {
      nextSubState = 4;
    }
    return true;
  }
  
  return false;
}

// Move position motor away from home switch
bool moveAwayFromPositionMotorHome(int& nextSubState) {
  PositionMotor_HOMING_settings();
  
  if (!positionMotor.isRunning()) {
    positionMotor.move(-300);  // Move 300 steps away
  }
  
  if (!readPositionMotorHomingSwitch() || positionMotor.distanceToGo() == 0) {
    positionMotor.stop();
    nextSubState = 4;
    return true;
  }
  
  return false;
}

// Home cut motor
bool homeCutMotor(int& nextSubState, ErrorType& errorType) {
  CutMotor_HOMING_settings();
  
  if (!cutMotor.isRunning() && !readCutMotorHomingSwitch()) {
    cutMotor.move(15000);  // Move enough steps to reach home
  }
  
  if (readCutMotorHomingSwitch()) {
    cutMotor.stop();
    cutMotor.setCurrentPosition(0);
    isCutMotorHomed = true;
    nextSubState = 5;
    return true;
  } else if (cutMotor.distanceToGo() == 0) {
    homingAttemptCount++;
    
    if (homingAttemptCount >= 3) {
      errorType = CUT_MOTOR_HOME_ERROR;
      return true;
    } else {
      nextSubState = 1;
      return true;
    }
  }
  
  return false;
}

// Home position motor
bool homePositionMotor(int& nextSubState, ErrorType& errorType) {
  PositionMotor_HOMING_settings();
  
  if (!positionMotor.isRunning() && !readPositionMotorHomingSwitch()) {
    positionMotor.move(10000);  // Move enough steps to reach home
  }
  
  if (readPositionMotorHomingSwitch()) {
    positionMotor.stop();
    positionMotor.setCurrentPosition(0);
    isPositionMotorHomed = true;
    return true;
  } else if (positionMotor.distanceToGo() == 0) {
    homingAttemptCount++;
    
    if (homingAttemptCount >= 3) {
      errorType = POSITION_MOTOR_HOME_ERROR;
      return true;
    } else {
      nextSubState = 3;
      return true;
    }
  }
  
  return false;
}

// Helper function to move away then home cut motor
void moveAwayThenHomeCutMotor() {
  static int subState = 0;
  static ErrorType errorType = NO_ERROR;
  
  switch (subState) {
    case 0:
      if (readCutMotorHomingSwitch()) {
        subState = 1;  // Need to move cut motor away from home
      } else {
        subState = 2;  // Already away from home
      }
      break;
      
    case 1:
      if (moveAwayFromCutMotorHome(subState)) {
        subState = 2;
      }
      break;
      
    case 2:
      if (homeCutMotor(subState, errorType)) {
        subState = 0;  // Reset for next use
      }
      break;
  }
}

// Helper function to move away then home position motor
void moveAwayThenHomePositionMotor() {
  static int subState = 0;
  static ErrorType errorType = NO_ERROR;
  
  switch (subState) {
    case 0:
      if (readPositionMotorHomingSwitch()) {
        subState = 1;  // Need to move position motor away from home
      } else {
        subState = 2;  // Already away from home
      }
      break;
      
    case 1:
      if (moveAwayFromPositionMotorHome(subState)) {
        subState = 2;
      }
      break;
      
    case 2:
      if (homePositionMotor(subState, errorType)) {
        subState = 0;  // Reset for next use
      }
      break;
  }
}

// Error handling function
void resetErrorAndHomeSystem() {
  static unsigned long startTime = 0;
  static int resetSubState = 0;
  static bool waitingForRelease = false;
  
  switch (resetSubState) {
    case 0:  // Wait for cycle switch press and release
      if (waitingForRelease) {
        if (!readCycleSwitch()) {
          waitingForRelease = false;
          resetSubState = 1;
        }
      } else if (readCycleSwitch()) {
        waitingForRelease = true;
      }
      break;
      
    case 1:  // Reset error and rehome
      initializeHomingVariables();
      resetSubState = 0;  // Reset for next error
      break;
  }
} 