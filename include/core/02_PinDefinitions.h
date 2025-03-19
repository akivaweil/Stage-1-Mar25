#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include <Arduino.h>

// Motor pins
extern const int CUT_MOTOR_PULSE_PIN;
extern const int CUT_MOTOR_DIR_PIN;
extern const int POSITION_MOTOR_PULSE_PIN;
extern const int POSITION_MOTOR_DIR_PIN;

// Switch and sensor pins
extern const int CUT_MOTOR_HOMING_SWITCH_PIN;
extern const int POSITION_MOTOR_HOMING_SWITCH_PIN;
extern const int RELOAD_SWITCH_PIN;
extern const int CYCLE_SWITCH_PIN;
extern const int YES_OR_NO_WOOD_SENSOR_PIN;
extern const int WAS_WOOD_SUCTIONED_SENSOR_PIN;

// Output control pins
extern const int POSITION_CLAMP_PIN;
extern const int WOOD_SECURE_CLAMP_PIN;
extern const int SIGNAL_TO_TRANSFER_ARM_PIN;

// LED pins
extern const int RED_LED_PIN;
extern const int YELLOW_LED_PIN;
extern const int GREEN_LED_PIN;
extern const int BLUE_LED_PIN;

#endif // PIN_DEFINITIONS_H 