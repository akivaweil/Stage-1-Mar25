#include "../../include/operations/13_NoWood.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/11_Cutting.h"

// Global variables for this module
static unsigned long noWoodWaitTime = 0;
static bool prevCycleSwitchState = false;

// Handle no wood state - indicates wood is no longer present after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0: // Step 7.1: Show NOWOOD indicator LEDs (Blue only)
      // Set LED indicators for NoWood path at the beginning of the state
      showNoWoodIndicator();
      
      subState = 1;
      break;
      
    case 1: // Step 7.2: Retract secure wood clamp
      retractWoodSecureClamp();
      
      subState = 2;
      break;
      
    case 2: // Step 7.3: Return both motors home
      // Set motors to return settings for faster return
      Motors_RETURN_settings();
      
      // Move both motors back to home position simultaneously
      moveCutMotorToPosition(0); // Move cut motor backward to home
      movePositionMotorToPosition(0);
      
      // When both motors are at home position, proceed to next step
      if (isMotorInPosition(cutMotor, 0) && isMotorInPosition(positionMotor, 0)) {
        subState = 3;
      }
      break;
      
    case 3: // Step 7.4: Release both clamps
      retractPositionClamp();
      retractWoodSecureClamp(); // Redundant but ensures it's retracted
      subState = 4;
      break;
      
    case 4: // Step 7.5: Set cycle switch toggle flag and wait for cycle switch toggle OFF
      // UNLIKE YESWOOD, this state REQUIRES cycle switch to be toggled
      needCycleSwitchToggle = true;
      
      // Check cycle switch state
      bool currentCycleSwitchState = readCycleSwitch();
      static bool prevSwitchState = currentCycleSwitchState;
      
      // If cycle switch was ON and is now OFF (toggled off)
      if (prevSwitchState && !currentCycleSwitchState) {
        // Return to READY state
        currentState = READY_STATE;
        subState = 0;
      }
      
      // Update previous state for next check
      prevSwitchState = currentCycleSwitchState;
      break;
      
    default:
      // Reset to initial substate if we get an invalid state
      subState = 0;
      break;
  }
}

// Show the NoWood state indicator (Blue LED only)
void showNoWoodIndicator() {
  setBlueLed(true);
  setGreenLed(false);
  setYellowLed(false);
  setRedLed(false);
} 