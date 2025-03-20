#include "../../include/operations/11_YesWoodOperations.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/operations/10_CuttingOperations.h"

// Global variables for this module
static unsigned long yesWoodWaitTime = 0;

// Handle yes wood state - indicates wood is present after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0:  // Signal transfer arm
      if (!hasTransferArmBeenSignaled) {
        // Move to position where we want to signal transfer arm
        movePositionMotorToPosition(TRANSFER_ARM_SIGNAL_POSITION);
        
        if (isMotorInPosition(positionMotor, TRANSFER_ARM_SIGNAL_POSITION * POSITION_MOTOR_STEPS_PER_INCH)) {
          signalTransferArm(HIGH);
          hasTransferArmBeenSignaled = true;
          subState = 1;  // Proceed to returning home
        }
      } else {
        subState = 1;  // Skip signaling if already done
      }
      break;
      
    case 1:  // Return motors to home
      returnToHomeAfterCut();
      
      // Proceed to showing indicator when motors are back home
      if (isMotorInPosition(cutMotor, 0) && isMotorInPosition(positionMotor, 0)) {
        subState = 2;
      }
      break;
      
    case 2:  // Show "Yes Wood" indicator and start timer
      showYesWoodIndicator();
      
      // Wait for 2 seconds to ensure operator sees the indication
      if (Wait(2000, &yesWoodWaitTime)) {
        subState = 3;
      }
      break;
      
    case 3:  // Wait for cycle switch to be toggled
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