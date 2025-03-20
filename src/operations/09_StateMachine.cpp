#include "../../include/operations/09_StateMachine.h"
#include "../../include/operations/10_HomingOperations.h"
#include "../../include/core/03_Utilities.h"

// State machine variables
unsigned long stateStartTime = 0;
unsigned long subStateTimer = 0;
int homingAttemptCount = 0;
bool isHomingComplete = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;

// State pattern implementation
typedef void (*StateHandler)();

// Array of function pointers - indexes match State enum values
StateHandler stateHandlers[] = {
  handleStartupState,
  handleHomingState,
  handleReadyState,
  handleReloadState,
  handleCuttingState,
  handleYesWoodState,
  handleNoWoodState,
  handleErrorState,
  handleWoodSuctionErrorState,
  handleCutMotorHomeErrorState,
  handlePositionMotorHomeErrorState
};

// Main state machine update function
void updateStateMachine() {
  // Direct lookup - no switch needed
  stateHandlers[currentState]();
}

// State transition function
void enterState(State newState) {
  // Exit actions for current state
  switch (currentState) {
    case CUTTING_STATE:
      signalTransferArm(LOW);
      break;
      
    case ERROR_STATE:
    case WOOD_SUCTION_ERROR_STATE:
    case CUT_MOTOR_HOME_ERROR_STATE:
    case POSITION_MOTOR_HOME_ERROR_STATE:
      setRedLed(false);
      break;
  }
  
  // Update state
  currentState = newState;
  stateStartTime = millis();
  subState = 0;  // Always reset subState when entering a new state
  subStateTimer = 0; // Reset substate timer as well
  
  // Entry actions for new state
  switch (newState) {
    case HOMING_STATE:
      initializeHomingVariables();
      break;
      
    case READY_STATE:
      setGreenLed(true);
      break;
      
    case RELOAD_STATE:
      setBlueLed(true);
      retractPositionClamp();
      retractWoodSecureClamp();
      break;
      
    case CUTTING_STATE:
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      break;
      
    case ERROR_STATE:
      currentError = NO_ERROR;
      break;
      
    case WOOD_SUCTION_ERROR_STATE:
      currentError = WOOD_SUCTION_ERROR;
      break;
      
    case CUT_MOTOR_HOME_ERROR_STATE:
      currentError = CUT_MOTOR_HOME_ERROR;
      break;
      
    case POSITION_MOTOR_HOME_ERROR_STATE:
      currentError = POSITION_MOTOR_HOME_ERROR;
      break;
  }
}

// Handle startup state - immediately transition to homing
void handleStartupState() {
  extendPositionClamp();
  extendWoodSecureClamp();
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
  enterState(HOMING_STATE);
}

// Handle homing state - delegates to the HomingOperations module
void handleHomingState() {
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

// Handle ready state - checks cycle switch toggle requirement
void handleReadyState() {
  setGreenLed(true);
  
  if (readReloadSwitch()) {
    enterState(RELOAD_STATE);
    return;
  }
  
  if (readCycleSwitch()) {
    if (needCycleSwitchToggle) {
      needCycleSwitchToggle = false;
      return;
    }
    
    enterState(CUTTING_STATE);
    return;
  }
  
  if (needCycleSwitchToggle && !readCycleSwitch()) {
    needCycleSwitchToggle = false;
  }
}

// Handle reload state - allows user to load new wood
void handleReloadState() {
  retractPositionClamp();
  retractWoodSecureClamp();
  setBlueLed(true);
  
  if (!readReloadSwitch()) {
    enterState(READY_STATE);
  }
}

// Handle error state
void handleErrorState() {
  updateRedLEDErrorPattern(currentError);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  ensureMotorsAtHome();
  
  if (cycleToggleDetected()) {
    resetErrorAndHomeSystem();
  }
}

// Handle wood suction error state
void handleWoodSuctionErrorState() {
  updateRedLEDErrorPattern(WOOD_SUCTION_ERROR);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  ensureMotorsAtHome();
  
  if (cycleToggleDetected()) {
    resetErrorAndHomeSystem();
  }
}

// Handle cut motor home error state
void handleCutMotorHomeErrorState() {
  updateRedLEDErrorPattern(CUT_MOTOR_HOME_ERROR);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  switch (subState) {
    case 0:  // First recovery attempt
      moveAwayThenHomeCutMotor();
      
      if (readCutMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 1;
      break;
      
    case 1:  // Second recovery attempt
      moveAwayThenHomeCutMotor();
      
      if (readCutMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 2;
      break;
      
    case 2:  // Locked state - requires cycle switch toggle
      bool currentCycleSwitchState = readCycleSwitch();
      
      if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
      }
      
      prevCycleSwitchState = currentCycleSwitchState;
      break;
  }
}

// Handle position motor home error state
void handlePositionMotorHomeErrorState() {
  updateRedLEDErrorPattern(POSITION_MOTOR_HOME_ERROR);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  switch (subState) {
    case 0:  // First recovery attempt
      moveAwayThenHomePositionMotor();
      
      if (readPositionMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 1;
      break;
      
    case 1:  // Second recovery attempt
      moveAwayThenHomePositionMotor();
      
      if (readPositionMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 2;
      break;
      
    case 2:  // Locked state - requires cycle switch toggle
      if (cycleToggleDetected()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
      }
      break;
  }
}

// Helper function for cut motor homing recovery attempts
void moveAwayThenHomeCutMotor() {
  static unsigned long startTime = 0;
  static int recoverySubState = 0;
  
  switch (recoverySubState) {
    case 0:  // Start moving away from home
      CutMotor_HOMING_settings();
      
      if (!cutMotor.isRunning()) {
        cutMotor.move(-500);  // Move 500 steps away
        startTime = millis();
        recoverySubState = 1;
      }
      break;
      
    case 1:  // Wait for move away to complete or timeout
      if (!cutMotor.isRunning() || (millis() - startTime > 2000)) {
        cutMotor.stop();
        recoverySubState = 2;
      }
      break;
      
    case 2:  // Start moving toward home
      CutMotor_HOMING_settings();
      
      if (!cutMotor.isRunning()) {
        cutMotor.move(2000);  // Move 2000 steps toward home
        recoverySubState = 3;
      }
      break;
      
    case 3:  // Wait for home position or movement to complete
      if (readCutMotorHomingSwitch()) {
        cutMotor.stop();
        cutMotor.setCurrentPosition(0);
        recoverySubState = 0;
      } else if (!cutMotor.isRunning()) {
        recoverySubState = 0;
      }
      break;
  }
}

// Helper function for position motor homing recovery attempts
void moveAwayThenHomePositionMotor() {
  static unsigned long startTime = 0;
  static int recoverySubState = 0;
  
  switch (recoverySubState) {
    case 0:  // Start moving away from home
      PositionMotor_HOMING_settings();
      
      if (!positionMotor.isRunning()) {
        positionMotor.move(-500);  // Move 500 steps away
        startTime = millis();
        recoverySubState = 1;
      }
      break;
      
    case 1:  // Wait for move away to complete or timeout
      if (!positionMotor.isRunning() || (millis() - startTime > 2000)) {
        positionMotor.stop();
        recoverySubState = 2;
      }
      break;
      
    case 2:  // Start moving toward home
      PositionMotor_HOMING_settings();
      
      if (!positionMotor.isRunning()) {
        positionMotor.move(2000);  // Move 2000 steps toward home
        recoverySubState = 3;
      }
      break;
      
    case 3:  // Wait for home position or movement to complete
      if (readPositionMotorHomingSwitch()) {
        positionMotor.stop();
        positionMotor.setCurrentPosition(0);
        recoverySubState = 0;
      } else if (!positionMotor.isRunning()) {
        recoverySubState = 0;
      }
      break;
  }
}

// Ensures both motors return to home position
void ensureMotorsAtHome() {
  // Return motors to home position if they're not already there
  if (!isMotorInPosition(cutMotor, 0)) {
    moveCutMotorToPosition(0);
  }
  
  if (!isMotorInPosition(positionMotor, 0)) {
    movePositionMotorToPosition(0);
  }
}

// Resets error state and returns to homing
void resetErrorAndHomeSystem() {
  currentError = NO_ERROR;
  enterState(HOMING_STATE);
} 