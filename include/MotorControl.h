#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "CommonDefinitions.h"
#include "PinDefinitions.h"
#include <AccelStepper.h>

// Motor instances
extern AccelStepper cutMotor;
extern AccelStepper positionMotor;

// Motor settings functions
void Motors_HOMING_settings();
void CutMotor_HOMING_settings();
void PositionMotor_HOMING_settings();
void Motors_NORMAL_settings();
void CutMotor_NORMAL_settings();
void PositionMotor_NORMAL_settings();
void Motors_RETURN_settings();
void CutMotor_RETURN_settings();
void PositionMotor_RETURN_settings();
void CutMotor_CUTTING_settings();

// Motor control functions
void configureMotors();
void runMotors();
void moveCutMotorToPosition(float positionInches);
void movePositionMotorToPosition(float positionInches);
bool isMotorInPosition(AccelStepper& motor, float targetPosition);
void ensureMotorsAtHome();

// Utility functions
float inchesToSteps(float inches, float stepsPerInch);
float stepsToInches(long steps, float stepsPerInch);

#endif // MOTOR_CONTROL_H 