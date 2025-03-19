#include "../../include/operations/12_NoWoodOperations.h"
#include "../../include/core/03_Utilities.h"

// Global variables for this module
static unsigned long noWoodWaitTime = 0;

// Handle no wood state - indicates wood is not present after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0:  // Show "No Wood" indicator
      showNoWoodIndicator();
      
      // Wait for 5 seconds to ensure operator sees the indication
      if (Wait(5000, &noWoodWaitTime)) {
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