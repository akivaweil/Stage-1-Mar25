#include "../../include/operations/12_YesWood.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/11_Cutting.h"

// Global variables for this module
static unsigned long cutMotorHomeCheckTimer = 0;
static bool positionMotorHomed = false;

// Handle yes wood state - indicates wood is present after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0: // Step 6.1: Show indicator LEDs first - Yellow only
      // Set LED indicators for YesWood path at the beginning of the state
      setYesWoodIndicator();
      
      // Reset position motor homed flag at the start of the state
      positionMotorHomed = false;
      
      subState = 1;
      break;
      
    case 1: // Step 6.2: Retract secure wood clamp
      retractWoodSecureClamp();
      extendPositionClamp(); // Ensure position clamp stays extended
      subState = 2;
      break;
      
    case 2: // Step 6.3: Move motors to home and control position clamp timing
      // Set motors to return settings for faster return
      Motors_RETURN_settings();
      
      // Move both motors back to home position simultaneously
      moveCutMotorToPosition(0);
      movePositionMotorToPosition(0);
      
      // Check if position motor has reached home
      if (isMotorInPosition(positionMotor, 0) && !positionMotorHomed) {
        // Position motor reached home, now retract position clamp immediately
        retractPositionClamp();
        positionMotorHomed = true;
      }
      
      // Once both motors have reached home, proceed to next step
      if (isMotorInPosition(cutMotor, 0) && isMotorInPosition(positionMotor, 0)) {
        subState = 3;
      }
      break;
      
    case 3: // Step 6.4: Extend position clamp immediately after retraction
      // Immediately extend position clamp
      extendPositionClamp();
      
      subState = 4;
      break;
      
    case 4: // Step 6.5: Check cut motor home sensor to detect slippage
      // Wait 100ms before checking the cut motor home sensor to allow for settling
      if (Wait(100, &cutMotorHomeCheckTimer)) {
        // Check if the cut motor is still at the home position
        if (!isCutMotorAtHome()) {
          // Cut motor is not at home - indicates potential slippage
          enterState(CUT_MOTOR_HOME_ERROR_STATE);
        } else {
          // Motor is properly homed, continue to next step
          subState = 5;
        }
      }
      break;
      
    case 5: // Step 6.6: Move position motor to 3.45" position
      movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE); // 3.45 inches
      
      // Once position reached, immediately transition to READY
      if (isMotorInPosition(positionMotor, POSITION_MOTOR_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH)) {
        // Reset flags for next cutting cycle
        hasTransferArmBeenSignaled = false;
        hasSuctionBeenChecked = false;
        
        // Ensure cycle switch toggle is NOT needed - allows continuous cycling
        needCycleSwitchToggle = false;
        
        // Return to ready state automatically with no delay
        enterState(READY_STATE);
      }
      break;
  }
}

// Show the YesWood state indicator (Yellow LED only)
void setYesWoodIndicator() {
  setYellowLed(true);
  setGreenLed(false);
  setBlueLed(false);
  setRedLed(false);
} 