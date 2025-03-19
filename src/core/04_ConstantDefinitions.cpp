#include "../../include/core/01_CommonDefinitions.h"

// Motor constants
const float CUT_MOTOR_STEPS_PER_INCH = 63.5;
const float POSITION_MOTOR_STEPS_PER_INCH = 1000.0;
const float CUT_MOTOR_TRAVEL_DISTANCE = 8.5;  // inches
const float POSITION_MOTOR_TRAVEL_DISTANCE = 3.45;  // inches
const float CUT_MOTOR_NORMAL_SPEED = 80.0;  // steps/sec
const float CUT_MOTOR_RETURN_SPEED = 2000.0;  // steps/sec
const float POSITION_MOTOR_NORMAL_SPEED = 30000.0;  // steps/sec
const float POSITION_MOTOR_RETURN_SPEED = 30000.0;  // steps/sec
const float CUT_MOTOR_ACCELERATION = 2200.0;  // steps/sec²
const float POSITION_MOTOR_ACCELERATION = 30000.0;  // steps/sec²
const float CUT_MOTOR_HOMING_SPEED = 300.0;  // steps/sec
const float POSITION_MOTOR_HOMING_SPEED = 2000.0;  // steps/sec

// Constants for cutting operations
const float CUT_MOTOR_CUTTING_SPEED = 50.0;  // steps/sec - slower speed for precise cutting
const float WAS_WOOD_SUCTIONED_POSITION = 0.3;  // inches
const float TRANSFER_ARM_SIGNAL_POSITION = 7.2;  // inches 