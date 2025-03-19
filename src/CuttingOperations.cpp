#include "../include/CuttingOperations.h"
#include "../include/StateMachine.h"
#include "../include/Utilities.h"
#include <Arduino.h>

// Configure cut motor for cutting operations (specific speed for cutting)
void CutMotor_CUTTING_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_CUTTING_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Handle cutting state - main cutting cycle
void handleCuttingState() {
  switch (subState) {
    case 0:  // Check wood suction
      if (!hasSuctionBeenChecked) {
        // Move to position where we can check suction
        movePositionMotorToPosition(WAS_WOOD_SUCTIONED_POSITION);
        
        if (isMotorInPosition(positionMotor, WAS_WOOD_SUCTIONED_POSITION * POSITION_MOTOR_STEPS_PER_INCH)) {
          hasSuctionBeenChecked = true;
          
          // Check if wood suction is proper
          if (!checkSuctionAndHandleErrors()) {
            // If there's an error, we're already in error state
            return;
          }
          
          subState = 1;  // Proceed to cutting position
        }
      } else {
        subState = 1;  // Skip suction check if already done
      }
      break;
      
    case 1:  // Move to cutting position
      moveToCuttingPosition();
      subState = 2;
      break;
      
    case 2:  // Perform the cut
      performCut();
      
      if (isCutComplete()) {
        subState = 3;  // Proceed to signaling
      }
      break;
      
    case 3:  // Signal transfer arm when cut is done
      if (!hasTransferArmBeenSignaled) {
        // Move to position where we want to signal transfer arm
        movePositionMotorToPosition(TRANSFER_ARM_SIGNAL_POSITION);
        
        if (isMotorInPosition(positionMotor, TRANSFER_ARM_SIGNAL_POSITION * POSITION_MOTOR_STEPS_PER_INCH)) {
          signalTransferArm(HIGH);
          hasTransferArmBeenSignaled = true;
          subState = 4;  // Proceed to check wood
        }
      } else {
        subState = 4;  // Skip signaling if already done
      }
      break;
      
    case 4:  // Return to home after cut & check wood
      returnToHomeAfterCut();
      
      // Transition when motors are back at home
      if (isMotorInPosition(cutMotor, 0) && isMotorInPosition(positionMotor, 0)) {
        transitionToWoodDetectionState();
      }
      break;
  }
}

// Check suction and handle errors
bool checkSuctionAndHandleErrors() {
  // If suction is not proper, transition to error state
  if (!isWoodSuctionProper()) {
    enterState(WOOD_SUCTION_ERROR_STATE);
    return false;
  }
  return true;
}

// Move to cutting position
void moveToCuttingPosition() {
  // Set position motor to move to cutting position
  movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE);
}

// Perform the cut
void performCut() {
  // Configure cut motor for cutting (slower speed)
  CutMotor_CUTTING_settings();
  
  if (!cutMotor.isRunning()) {
    // Move cut motor to its full travel distance
    moveCutMotorToPosition(CUT_MOTOR_TRAVEL_DISTANCE);
  }
}

// Check if cut is complete
bool isCutComplete() {
  return isMotorInPosition(cutMotor, CUT_MOTOR_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
}

// Return to home after cutting
void returnToHomeAfterCut() {
  // Set both motors to return settings for faster return
  Motors_RETURN_settings();
  
  // Move motors back to home position
  moveCutMotorToPosition(0);
  movePositionMotorToPosition(0);
}

// Transition to wood detection state
void transitionToWoodDetectionState() {
  // Check if wood is present and transition to appropriate state
  if (isWoodPresent()) {
    enterState(YESWOOD_STATE);
  } else {
    enterState(NOWOOD_STATE);
  }
}

// Move cut motor to specified position in inches
void moveCutMotorToPosition(float positionInches) {
  long steps = positionInches * CUT_MOTOR_STEPS_PER_INCH;
  cutMotor.moveTo(steps);
}

// Check if motor has reached target position with small tolerance
bool isMotorInPosition(AccelStepper& motor, float targetPosition) {
  return (abs(motor.currentPosition() - targetPosition) < 5) && !motor.isRunning();
} 