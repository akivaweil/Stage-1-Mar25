#include "../../include/operations/12_NoWoodOperations.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/10_CuttingOperations.h"

// Global variables for this module
static unsigned long noWoodWaitTime = 0;

// Handle no wood state - indicates wood is not present after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0: // Step 1: Release secure wood clamp and keep position clamp extended
      retractWoodSecureClamp();
      subState = 1;
      break;
      
    case 1: // Step 2: Return both motors home
      // Set motors to return settings for faster return
      Motors_RETURN_settings();
      
      // Move motors back to home position
      moveCutMotorToPosition(0);
      movePositionMotorToPosition(0);
      
      // When both motors are at home position, proceed to next step
      if (isMotorInPosition(cutMotor, 0) && isMotorInPosition(positionMotor, 0)) {
        subState = 2;
      }
      break;
      
    case 2: // Step 3: Release both clamps
      retractPositionClamp();
      retractWoodSecureClamp(); // Redundant but ensures it's retracted
      subState = 3;
      break;
      
    case 3: // Step 4: Set flag for cycle switch toggle requirement and show indicator
      // Set the flag that requires cycle switch to be toggled
      needCycleSwitchToggle = true;
      
      // Show the NoWood indicator
      showNoWoodIndicator();
      
      // Wait for 1 second to ensure operator sees the indication
      if (Wait(1000, &noWoodWaitTime)) {
        // Reset flags for next cutting cycle
        hasTransferArmBeenSignaled = false;
        hasSuctionBeenChecked = false;
        
        // Return to ready state instead of reload
        enterState(READY_STATE);
      }
      break;
  }
}

// Show no wood indicator (Green + Blue LEDs)
void showNoWoodIndicator() {
  setGreenLed(true);
  setBlueLed(true);
} 