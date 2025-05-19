#ifndef GLOBAL_PIN_DEFINITIONS_H
#define GLOBAL_PIN_DEFINITIONS_H

// It's good practice to include Arduino.h if using pin numbers, though not strictly necessary for simple integer defines.
#include <Arduino.h>

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 12
#define CUT_MOTOR_DIR_PIN 11
#define POSITION_MOTOR_PULSE_PIN 17
#define POSITION_MOTOR_DIR_PIN 18

// Switch and Sensor Pin Definitions
#define CUT_MOTOR_HOMING_SWITCH_PIN 3
#define POSITION_MOTOR_HOMING_SWITCH_PIN 16
#define RELOAD_SWITCH_PIN 6
#define CYCLE_SWITCH_PIN 5
#define YES_OR_NO_WOOD_SENSOR_PIN 10
#define WAS_WOOD_SUCTIONED_SENSOR_PIN 37
// CUT_MOTOR_EMERGENCY_SWITCH_PIN was removed as per user request

// Clamp Pin Definitions
#define POSITION_CLAMP_PIN 36
#define WOOD_SECURE_CLAMP_PIN 35

// Signal Pin Definition
#define SIGNAL_TO_TRANSFER_ARM_PIN 19

// LED Pin Definitions
#define RED_LED_PIN 45
#define YELLOW_LED_PIN 48
#define GREEN_LED_PIN 47
#define BLUE_LED_PIN 21

#endif // GLOBAL_PIN_DEFINITIONS_H 