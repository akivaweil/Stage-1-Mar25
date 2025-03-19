#include "../include/NoWoodOperations.h"

// Handle no wood state - no wood was detected after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0:  // Prepare for return to home
      retractPositionClamp();
      retractWoodSecureClamp();
      Motors_RETURN_settings();
      subState = 1;
      break;
      
    case 1:  // Return cut motor to home
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(0);
      }
      
      if (isMotorInPosition(cutMotor, 0)) {
        subState = 2;
      }
      break;
      
    case 2:  // Return position motor to home
      if (!positionMotor.isRunning()) {
        movePositionMotorToPosition(0);
      }
      
      if (isMotorInPosition(positionMotor, 0)) {
        enterState(READY_STATE);
      }
      break;
  }
} 