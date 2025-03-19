#include "../../include/operations/11_YesWoodOperations.h"
#include "../../include/core/03_Utilities.h"

// Global variables for this module
static unsigned long yesWoodWaitTime = 0;

// Handle yes wood state - indicates wood is present after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0:  // Show "Yes Wood" indicator and start timer
      showYesWoodIndicator();
      
      // Wait for 2 seconds to ensure operator sees the indication
      if (Wait(2000, &yesWoodWaitTime)) {
        subState = 1;
      }
      break;
      
    case 1:  // Wait for cycle switch to be toggled
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
    // Return to ready state when cycle switch is toggled
    enterState(READY_STATE);
  }
} 