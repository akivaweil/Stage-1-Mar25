#include "../include/MotorControl.h"

// Motor instances
AccelStepper cutMotor(AccelStepper::DRIVER, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(AccelStepper::DRIVER, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

// Configure motors with their speeds and acceleration
void configureMotors() {
  Motors_NORMAL_settings();
  
  cutMotor.setCurrentPosition(0);
  positionMotor.setCurrentPosition(0);
}

// Run motors - call this in each loop iteration for non-blocking operation
void runMotors() {
  cutMotor.run();
  positionMotor.run();
}

// Convert inches to motor steps
float inchesToSteps(float inches, float stepsPerInch) {
  return inches * stepsPerInch;
}

// Convert motor steps to inches
float stepsToInches(long steps, float stepsPerInch) {
  return (float)steps / stepsPerInch;
}

// Move cut motor to specified position in inches
void moveCutMotorToPosition(float positionInches) {
  long steps = positionInches * CUT_MOTOR_STEPS_PER_INCH;
  cutMotor.moveTo(steps);
}

// Move position motor to specified position in inches
void movePositionMotorToPosition(float positionInches) {
  long steps = positionInches * POSITION_MOTOR_STEPS_PER_INCH;
  positionMotor.moveTo(steps);
}

// Check if motor has reached target position with small tolerance
bool isMotorInPosition(AccelStepper& motor, float targetPosition) {
  return (abs(motor.currentPosition() - targetPosition) < 5) && !motor.isRunning();
}

// Ensures both motors return to home position
void ensureMotorsAtHome() {
  // Return motors to home position if they're not already there
  if (!isMotorInPosition(cutMotor, 0)) {
    moveCutMotorToPosition(0);
  }
  
  if (!isMotorInPosition(positionMotor, 0)) {
    movePositionMotorToPosition(0);
  }
}

// Configure both motors for homing operations
void Motors_HOMING_settings() {
  CutMotor_HOMING_settings();
  PositionMotor_HOMING_settings();
}

// Configure cut motor for homing
void CutMotor_HOMING_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_HOMING_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for homing
void PositionMotor_HOMING_settings() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_HOMING_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure both motors for normal operations
void Motors_NORMAL_settings() {
  CutMotor_NORMAL_settings();
  PositionMotor_NORMAL_settings();
}

// Configure cut motor for normal operations
void CutMotor_NORMAL_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for normal operations
void PositionMotor_NORMAL_settings() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure both motors for return operations
void Motors_RETURN_settings() {
  CutMotor_RETURN_settings();
  PositionMotor_RETURN_settings();
}

// Configure cut motor for return operations
void CutMotor_RETURN_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for return operations
void PositionMotor_RETURN_settings() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure cut motor for cutting operations (specific speed for cutting)
void CutMotor_CUTTING_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_CUTTING_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
} 