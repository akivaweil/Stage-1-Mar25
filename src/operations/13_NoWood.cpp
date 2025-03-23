#include "../../include/operations/13_NoWood.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/11_Cutting.h"

// Global variables for this module
static unsigned long noWoodWaitTime = 0;
static bool prevCycleSwitchState = false;

// Handle no wood state - indicates wood is not present after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0: // Step 7.1: Show indicator LEDs first - Green + Blue
      // Show the NoWood indicator - Green and Blue LEDs
      setBlueLed(true);
      setGreenLed(true);
      setYellowLed(false);
      setRedLed(false);
      subState = 1;
      break;

    case 1: // Step 7.2: Retract secure wood clamp
      retractWoodSecureClamp();
      extendPositionClamp(); // Ensure position clamp stays extended
      subState = 2;
      break;
      
    case 2: // Step 7.3: Return both motors home simultaneously
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
      
    case 4: // Step 7.5: Set cycle switch toggle flag
      // Set the flag that requires cycle switch to be toggled
      needCycleSwitchToggle = true;
      prevCycleSwitchState = readCycleSwitch();
      subState = 5;
      break;
      
    case 5: // Step 7.6: Wait for cycle switch to be flipped off and return to READY
      // Check cycle switch state
      bool currentCycleSwitchState = readCycleSwitch();
      
      // If cycle switch was on and is now off (toggled off)
      if (prevCycleSwitchState == true && currentCycleSwitchState == false) {
        // Reset flags for next cutting cycle
        hasTransferArmBeenSignaled = false;
        hasSuctionBeenChecked = false;
        needCycleSwitchToggle = false;
        
        // Return to ready state
        enterState(READY_STATE);
      }
      
      // Update previous state for next check
      prevCycleSwitchState = currentCycleSwitchState;
      break;
  }
}

// Show no wood indicator (Blue LED only)
void showNoWoodIndicator() {
  setBlueLed(true);
  setGreenLed(false);
  setYellowLed(false);
  setRedLed(false);
} 