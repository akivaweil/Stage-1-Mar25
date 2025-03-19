#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

#include <Arduino.h>
#include <AccelStepper.h>

// Enumerations
enum State {
  STARTUP_STATE,
  HOMING_STATE,
  READY_STATE,
  RELOAD_STATE,
  CUTTING_STATE,
  YESWOOD_STATE,
  NOWOOD_STATE,
  ERROR_STATE,
  WOOD_SUCTION_ERROR_STATE,
  CUT_MOTOR_HOME_ERROR_STATE,
  POSITION_MOTOR_HOME_ERROR_STATE
};

enum ErrorType {
  NO_ERROR,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR,
  POSITION_MOTOR_HOME_ERROR
};

// Motor constants
extern const float CUT_MOTOR_STEPS_PER_INCH;
extern const float POSITION_MOTOR_STEPS_PER_INCH;
extern const float CUT_MOTOR_TRAVEL_DISTANCE;
extern const float POSITION_MOTOR_TRAVEL_DISTANCE;
extern const float CUT_MOTOR_NORMAL_SPEED;
extern const float CUT_MOTOR_RETURN_SPEED;
extern const float POSITION_MOTOR_NORMAL_SPEED;
extern const float POSITION_MOTOR_RETURN_SPEED;
extern const float CUT_MOTOR_ACCELERATION;
extern const float POSITION_MOTOR_ACCELERATION;
extern const float CUT_MOTOR_HOMING_SPEED;
extern const float POSITION_MOTOR_HOMING_SPEED;

// External references to shared variables
extern State currentState;
extern ErrorType currentError;
extern int subState;
extern bool hasSuctionBeenChecked;
extern bool hasTransferArmBeenSignaled;
extern AccelStepper cutMotor;
extern AccelStepper positionMotor;

// Common function declarations
void enterState(State newState);
bool isMotorInPosition(AccelStepper& motor, float targetPosition);
void moveCutMotorToPosition(float positionInches);
void movePositionMotorToPosition(float positionInches);
void extendPositionClamp();
void retractPositionClamp();
void extendWoodSecureClamp();
void retractWoodSecureClamp();
bool isWoodSuctionProper();
bool isWoodPresent();
void signalTransferArm(bool state);

#endif // COMMON_DEFINITIONS_H 