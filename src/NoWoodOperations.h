#ifndef NOWOOD_OPERATIONS_H
#define NOWOOD_OPERATIONS_H

#include <Arduino.h>
#include <AccelStepper.h>

// External references to variables/functions from main file
extern AccelStepper cutMotor;
extern AccelStepper positionMotor;
extern float POSITION_MOTOR_STEPS_PER_INCH;
extern int subState;

// Forward declaration of State enum
enum State : int;
// Forward declaration for target state
extern State READY_STATE;

// Function declarations
void handleNoWoodState();
void Motors_RETURN_settings();
void movePositionMotorToPosition(float positionInches);
bool isMotorInPosition(AccelStepper& motor, float targetPosition);

// External function references needed
extern void retractPositionClamp();
extern void retractWoodSecureClamp();
extern void moveCutMotorToPosition(float positionInches);
extern void enterState(State newState);

#endif // NOWOOD_OPERATIONS_H 