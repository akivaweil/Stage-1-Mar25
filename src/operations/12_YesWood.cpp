#include "../../include/operations/12_YesWood.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/11_Cutting.h"

// Global variables for this module
static unsigned long yesWoodWaitTime = 0;
static unsigned long cutMotorHomeCheckTimer = 0;

// Handle yes wood state - indicates wood is present after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0: // Step A7: Set LED Indicators - yellow LED ON, others OFF
      // Set LED indicators for YesWood path
      setYellowLed(true);
      setGreenLed(false);
      setBlueLed(false);
      setRedLed(false);
      
      subState = 1;
      break;
      
    case 1: // Step A8: Retract cut motor to home
      // Set motors to return settings for faster return
      Motors_RETURN_settings();
      
      // Move cut motor back to home position
      moveCutMotorToPosition(0);
      
      if (isMotorInPosition(cutMotor, 0)) {
        subState = 2;
      }
      break;
      
    case 2: // Step A9: Retract secure wood clamp (while keeping position clamp extended)
      retractWoodSecureClamp();
      extendPositionClamp(); // Ensure position clamp stays extended
      subState = 3;
      break;
      
    case 3: // Step A10: Position motor moves to home
      movePositionMotorToPosition(0);
      
      if (isMotorInPosition(positionMotor, 0)) {
        subState = 4;
      }
      break;
      
    case 4: // Step A11: Adjust clamps at home position
      // Retract position clamp
      retractPositionClamp();
      
      // Extend secure wood clamp
      extendWoodSecureClamp();
      
      subState = 5;
      break;
      
    case 5: // Step A12: Extend position clamp
      // Immediately extend position clamp
      extendPositionClamp();
      
      subState = 6;
      break;
      
    case 6: // Step A13: Verify cut motor at home
      // Start timer for 50ms check
      if (cutMotorHomeCheckTimer == 0) {
        cutMotorHomeCheckTimer = millis();
      }
      
      // Check cut motor homing sensor after 50ms
      if (millis() - cutMotorHomeCheckTimer >= 50) {
        // If homing sensor is not active, enter error state
        if (!readCutMotorHomingSwitch()) {
          enterState(CUT_MOTOR_HOME_ERROR_STATE);
          return;
        }
        
        // Reset timer
        cutMotorHomeCheckTimer = 0;
        
        // Proceed to next step
        subState = 7;
      }
      break;
      
    case 7: // Step A14: Move position motor to 3.45 inches position
      movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE); // 3.45 inches
      
      // Once position reached, immediately transition to READY
      if (isMotorInPosition(positionMotor, POSITION_MOTOR_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH)) {
        // Reset flags for next cutting cycle
        hasTransferArmBeenSignaled = false;
        hasSuctionBeenChecked = false;
        
        // Ensure cycle switch toggle is not needed - allows continuous cycling
        needCycleSwitchToggle = false;
        
        // Return to ready state automatically with no delay
        enterState(READY_STATE);
      }
      break;
  }
}

// Show yes wood indicator (Yellow LED only)
void showYesWoodIndicator() {
  setYellowLed(true);
  setGreenLed(false);
  setBlueLed(false);
  setRedLed(false);
}

// Wait for cycle switch to be toggled to continue
void waitForCycleSwitch() {
  if (cycleToggleDetected()) {
    // Reset flags for next cutting cycle
    hasTransferArmBeenSignaled = false;
    hasSuctionBeenChecked = false;
    
    // Return to ready state when cycle switch is toggled
    enterState(READY_STATE);
  }
} 