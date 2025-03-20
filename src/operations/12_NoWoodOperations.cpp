#include "../../include/operations/12_NoWoodOperations.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/10_CuttingOperations.h"

// Global variables for this module
static unsigned long noWoodWaitTime = 0;

// Handle no wood state - indicates wood is not present after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0:  // Return motors to home
      returnToHomeAfterCut();
      
      // Proceed to showing indicator when motors are back home
      if (isMotorInPosition(cutMotor, 0) && isMotorInPosition(positionMotor, 0)) {
        subState = 1;
      }
      break;
      
    case 1:  // Show "No Wood" indicator
      showNoWoodIndicator();
      
      // Wait for 5 seconds to ensure operator sees the indication
      if (Wait(5000, &noWoodWaitTime)) {
        // Reset flags for next cutting cycle
        hasTransferArmBeenSignaled = false;
        hasSuctionBeenChecked = false;
        
        // Return to reload state to load new wood
        enterState(RELOAD_STATE);
      }
      break;
  }
}

// Show no wood indicator (Green + Blue LEDs)
void showNoWoodIndicator() {
  setGreenLed(true);
  setBlueLed(true);
} 