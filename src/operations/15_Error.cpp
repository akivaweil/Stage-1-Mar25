#include "../../include/operations/15_Error.h"
#include "../../include/core/03_Utilities.h"

// Update error LED pattern based on error type
void updateErrorLED(ErrorType errorType) {
  static unsigned long errorLedTimer = 0;
  static int blinkCount = 0;
  static bool isLedOn = false;
  
  // Use Wait for non-blocking timing
  if (Wait(250, &errorLedTimer)) {
    if (isLedOn) {
      setRedLed(false);
      isLedOn = false;
      blinkCount++;
      
      // Reset after complete pattern - number of blinks corresponds to error code
      if (blinkCount > (int)errorType * 2) {
        blinkCount = 0;
        errorLedTimer = 0;  // Add delay between patterns
        return;
      }
    } else {
      setRedLed(true);
      isLedOn = true;
    }
  }
}

// Reset error state
void resetErrorState() {
  currentError = NO_ERROR;
  setRedLed(false);
} 