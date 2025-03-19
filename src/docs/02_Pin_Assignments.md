# Pin Assignments

## Motor Control Pins
- Cut Motor Pulse Pin: 48
- Cut Motor Direction Pin: 47
- Position Motor Pulse Pin: 21
- Position Motor Direction Pin: 20

## Switch and Sensor Pins
- Cut Motor Position Switch Pin: 10
- Position Motor Position Switch Pin: 9
- Reload Switch Pin: 14
- Cycle Switch Pin: 13
- YESorNO_WOOD_SENSOR Pin: 11
- WAS_WOOD_SUCTIONED_SENSOR Pin: 8

## Clamp Pins
- Position Clamp Pin: 18
- Wood Secure Clamp Pin: 17

## LED Pins
- Red LED Pin: 7
- Yellow LED Pin: 6
- Green LED Pin: 16
- Blue LED Pin: 15

## Signal Output
- Signal to Transfer Arm (TA) Pin: 19

## Pin Definitions in Code

```cpp
// Motor control pins
const int CUT_MOTOR_PULSE_PIN = 48;
const int CUT_MOTOR_DIR_PIN = 47;
const int POSITION_MOTOR_PULSE_PIN = 21;
const int POSITION_MOTOR_DIR_PIN = 20;

// Switch and sensor pins
const int CUT_MOTOR_POSITION_SWITCH_PIN = 10;
const int POSITION_MOTOR_POSITION_SWITCH_PIN = 9;
const int RELOAD_SWITCH_PIN = 14;
const int CYCLE_SWITCH_PIN = 13;
const int YES_OR_NO_WOOD_SENSOR_PIN = 11;
const int WAS_WOOD_SUCTIONED_SENSOR_PIN = 8;

// Clamp pins
const int POSITION_CLAMP_PIN = 18;
const int WOOD_SECURE_CLAMP_PIN = 17;

// LED pins
const int RED_LED_PIN = 7;
const int YELLOW_LED_PIN = 6;
const int GREEN_LED_PIN = 16;
const int BLUE_LED_PIN = 15;

// Signal output
const int SIGNAL_TO_TRANSFER_ARM_PIN = 19;
``` 