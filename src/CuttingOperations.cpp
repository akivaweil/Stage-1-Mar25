#include "../include/CuttingOperations.h"
#include <Arduino.h>

// Configure cut motor for cutting operations (specific speed for cutting)
void CutMotor_CUTTING_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_CUTTING_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Handle cutting state - implements a non-blocking cutting sequence
void handleCuttingState() {
  static bool hasPastSuctionPosition = false;
  static bool hasPastTransferArmPosition = false;
  
  switch (subState) {
    case 0:  // Init cutting cycle
      extendPositionClamp();
      extendWoodSecureClamp();
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      hasPastSuctionPosition = false;
      hasPastTransferArmPosition = false;
      
      // Set up for continuous motion to final position
      CutMotor_CUTTING_settings();
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(CUT_MOTOR_TRAVEL_DISTANCE);
      }
      
      subState = 1;
      break;
      
    case 1:  // Monitor position during continuous movement and trigger actions at specific points
      // Check if we've passed the suction check position
      if (!hasSuctionBeenChecked && !hasPastSuctionPosition && 
          cutMotor.currentPosition() >= WAS_WOOD_SUCTIONED_POSITION * CUT_MOTOR_STEPS_PER_INCH) {
        
        hasPastSuctionPosition = true;
        
        // Check suction without stopping the motor
        if (isWoodSuctionProper()) {
          hasSuctionBeenChecked = true;
        } else {
          // Stop the motor and transition to error state if suction fails
          cutMotor.stop();
          enterState(WOOD_SUCTION_ERROR_STATE);
          return;
        }
      }
      
      // Check if we've passed the transfer arm signal position
      if (hasSuctionBeenChecked && !hasTransferArmBeenSignaled && !hasPastTransferArmPosition && 
          cutMotor.currentPosition() >= TRANSFER_ARM_SIGNAL_POSITION * CUT_MOTOR_STEPS_PER_INCH) {
        
        hasPastTransferArmPosition = true;
        
        // Signal transfer arm without stopping the motor
        signalTransferArm(HIGH);
        hasTransferArmBeenSignaled = true;
      }
      
      // Check if we've completed the full travel
      if (isMotorInPosition(cutMotor, CUT_MOTOR_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH)) {
        subState = 2;
      }
      break;
      
    case 2:  // Check wood presence after completing the cut
      if (isWoodPresent()) {
        enterState(YESWOOD_STATE);
      } else {
        enterState(NOWOOD_STATE);
      }
      break;
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