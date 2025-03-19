#ifndef CUTTING_OPERATIONS_H
#define CUTTING_OPERATIONS_H

#include <Arduino.h>
#include <AccelStepper.h>

// External references to variables/functions from main file
extern AccelStepper cutMotor;
extern float CUT_MOTOR_STEPS_PER_INCH;
extern float CUT_MOTOR_TRAVEL_DISTANCE;
extern float CUT_MOTOR_ACCELERATION;
extern int subState;
extern bool hasSuctionBeenChecked;
extern bool hasTransferArmBeenSignaled;

// Forward declaration of State enum for use in function declarations
enum State : int;
// Forward declaration for error state
extern State WOOD_SUCTION_ERROR_STATE;
extern State YESWOOD_STATE;
extern State NOWOOD_STATE;

// Constants for cutting operations
extern const float CUT_MOTOR_CUTTING_SPEED;
extern const float WAS_WOOD_SUCTIONED_POSITION;
extern const float TRANSFER_ARM_SIGNAL_POSITION;

// Function declarations
void CutMotor_CUTTING_settings();
void handleCuttingState();
void moveCutMotorToPosition(float positionInches);
bool isMotorInPosition(AccelStepper& motor, float targetPosition);

// External function references needed by cutting operations
extern void extendPositionClamp();
extern void extendWoodSecureClamp();
extern bool isWoodSuctionProper();
extern bool isWoodPresent();
extern void signalTransferArm(bool state);
extern void enterState(State newState);

#endif // CUTTING_OPERATIONS_H 