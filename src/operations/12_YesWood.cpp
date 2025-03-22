#include "../../include/operations/12_YesWood.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/11_Cutting.h"

// Global variables for this module
static unsigned long yesWoodWaitTime = 0;
static unsigned long cutMotorHomeCheckTimer = 0;

// Handle yes wood state - indicates wood is present after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0: // Step 1: Retract secure wood clamp (while keeping position clamp extended)
      retractWoodSecureClamp();
      subState = 1;
      break;
      
    case 1: // Step 2: Tell both motors to home
      // Set motors to return settings for faster return
      Motors_RETURN_settings();
      
      // Move motors back to home position
      moveCutMotorToPosition(0); // Move cut motor backward to home
      movePositionMotorToPosition(0);
      
      subState = 2;
      break;
      
    case 2: // Step 3: Track position motor and when it reaches 3.35 inches, adjust clamps
      if (stepsToInches(positionMotor.currentPosition(), POSITION_MOTOR_STEPS_PER_INCH) <= 3.35) {
        // Retract position clamp
        retractPositionClamp();
        
        // Extend secure wood clamp
        extendWoodSecureClamp();
        
        subState = 3;
      }
      break;
      
    case 3: // Step 4: When position motor reaches home, extend position clamp
      if (isMotorInPosition(positionMotor, 0)) {
        // Immediately extend position clamp
        extendPositionClamp();
        
        subState = 4;
      }
      break;
      
    case 4: // Step 5: Wait for cut motor to reach home, then check homing sensor
      if (isMotorInPosition(cutMotor, 0)) {
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
          subState = 5;
        }
      }
      break;
      
    case 5: // Step 6: Move position motor to 3.45 inches position
      movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE); // 3.45 inches
      
      // Once position reached, proceed to showing indicator
      if (isMotorInPosition(positionMotor, POSITION_MOTOR_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH)) {
        subState = 6;
      }
      break;
      
    case 6: // Show "Yes Wood" indicator and start timer
      showYesWoodIndicator();
      
      // Wait for 2 seconds to ensure operator sees the indication
      if (Wait(2000, &yesWoodWaitTime)) {
        subState = 7;
      }
      break;
      
    case 7: // Wait for cycle switch to be toggled
      waitForCycleSwitch();
      break;
  }
}

// Show yes wood indicator (Green + Yellow LEDs)
void showYesWoodIndicator() {
  setGreenLed(true);
  setYellowLed(true);
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