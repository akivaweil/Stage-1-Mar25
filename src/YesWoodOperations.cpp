#include "../include/YesWoodOperations.h"

// Handle yes wood state - wood was detected after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0:  // Prepare for next position
      retractPositionClamp();
      PositionMotor_NORMAL_settings();
      subState = 1;
      break;
      
    case 1:  // Move position motor for next cut
      if (!positionMotor.isRunning()) {
        movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE);
      }
      
      if (isMotorInPosition(positionMotor, POSITION_MOTOR_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH)) {
        subState = 2;
      }
      break;
      
    case 2:  // Return cut motor to home
      CutMotor_RETURN_settings();
      
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(0);
      }
      
      if (isMotorInPosition(cutMotor, 0)) {
        enterState(READY_STATE);
      }
      break;
  }
} 