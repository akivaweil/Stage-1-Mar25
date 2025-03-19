# Motor Configuration

## Motor Calibration
- Cut Motor: 63.5 steps per inch
- Position Motor: 1000 steps per inch
- Cut Motor Travel Distance: 8.5 inches
- Position Motor Travel Distance: 3.45 inches

## Motor Speeds
- Cut Motor Normal Speed: 80 steps/sec
- Cut Motor Return Speed: 2000 steps/sec
- Position Motor Normal Speed: 30000 steps/sec
- Position Motor Return Speed: 30000 steps/sec

## Motor Acceleration
- Cut Motor Acceleration: 2200 steps/sec²
- Position Motor Acceleration: 30000 steps/sec²
- Position Motor Return Acceleration: 30000 steps/sec²

## Homing Speeds
- Cut Motor Homing Speed: 300 steps/sec
- Position Motor Homing Speed: 2000 steps/sec

## Clamp Operation
- LOW signal = Clamp Extended/Engaged
- HIGH signal = Clamp Retracted

## Motor Constants in Code

```cpp
// Motor calibration constants
const float CUT_MOTOR_STEPS_PER_INCH = 63.5;
const float POSITION_MOTOR_STEPS_PER_INCH = 1000.0;

// Travel distances
const float CUT_MOTOR_TRAVEL_DISTANCE = 8.5;  // inches
const float POSITION_MOTOR_TRAVEL_DISTANCE = 3.45;  // inches

// Motor speeds
const float CUT_MOTOR_NORMAL_SPEED = 80.0;  // steps/sec
const float CUT_MOTOR_RETURN_SPEED = 2000.0;  // steps/sec
const float POSITION_MOTOR_NORMAL_SPEED = 30000.0;  // steps/sec
const float POSITION_MOTOR_RETURN_SPEED = 30000.0;  // steps/sec

// Motor acceleration
const float CUT_MOTOR_ACCELERATION = 2200.0;  // steps/sec²
const float POSITION_MOTOR_ACCELERATION = 30000.0;  // steps/sec²
const float POSITION_MOTOR_RETURN_ACCELERATION = 30000.0;  // steps/sec²

// Homing speeds
const float CUT_MOTOR_HOMING_SPEED = 300.0;  // steps/sec
const float POSITION_MOTOR_HOMING_SPEED = 2000.0;  // steps/sec

// Specific positions
const float WOOD_SUCTION_CHECK_POSITION = 0.3;  // inches
const float TRANSFER_ARM_SIGNAL_POSITION = 7.2;  // inches
``` 