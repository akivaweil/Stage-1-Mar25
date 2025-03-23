#include "../../include/operations/12_YesWood.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/11_Cutting.h"

// Global variables for this module
static unsigned long yesWoodWaitTime = 0;
static unsigned long cutMotorHomeCheckTimer = 0;

// Handle yes wood state - indicates wood is present after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0: // Step 6.1: Show indicator LEDs first - Green + Yellow
      // Set LED indicators for YesWood path at the beginning of the state
      setYellowLed(true);
      setGreenLed(true);
      setBlueLed(false);
      setRedLed(false);
      
      subState = 1;
      break;
      
    case 1: // Step 6.2: Retract secure wood clamp
      retractWoodSecureClamp();
      extendPositionClamp(); // Ensure position clamp stays extended
      subState = 2;
      break;
      
    case 2: // Step 6.3: Move motors to home
      // Set motors to return settings for faster return
      Motors_RETURN_settings();
      
      // Move both motors back to home position simultaneously
      moveCutMotorToPosition(0);
      movePositionMotorToPosition(0);
      
      if (isMotorInPosition(cutMotor, 0) && isMotorInPosition(positionMotor, 0)) {
        subState = 3;
      }
      break;
      
    case 3: // Step 6.4: Retract position clamp at 3.35 inches
      // Retract position clamp
      retractPositionClamp();
      
      subState = 4;
      break;
      
    case 4: // Step 6.5: Extend position clamp immediately when position motor reaches home
      // Immediately extend position clamp
      extendPositionClamp();
      
      subState = 5;
      break;
      
    case 5: // Step 6.6: Check cut motor homing sensor 100ms after reaching home
      // Start timer for 100ms check
      if (cutMotorHomeCheckTimer == 0) {
        cutMotorHomeCheckTimer = millis();
      }
      
      // Check cut motor homing sensor after 100ms
      if (millis() - cutMotorHomeCheckTimer >= 100) {
        // If homing sensor is not active, enter error state
        if (!readCutMotorHomingSwitch()) {
          enterState(CUT_MOTOR_HOME_ERROR_STATE);
          return;
        }
        
        // Reset timer
        cutMotorHomeCheckTimer = 0;
        
        // Proceed to next step
        subState = 6;
      }
      break;
      
    case 6: // Step 6.7: Move position motor to 3.45" position
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