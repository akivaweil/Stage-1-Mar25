# Motor Configuration

This document describes the configuration parameters for the stepper motors used in the wood cutting machine.

## Non-Blocking Operation Requirement
- All motor operations must be non-blocking
- No Serial.print statements during motor movements
- Motors must use the AccelStepper library for smooth acceleration/deceleration

## Motor Calibration

### Cut Motor
- **Steps per Inch**: 63.5 steps/inch
- **Total Travel**: 8.5 inches
- **Position Switch**: Active HIGH (triggered when reading HIGH)

### Position Motor
- **Steps per Inch**: 1000 steps/inch
- **Total Travel**: 3.45 inches
- **Position Switch**: Active HIGH (triggered when reading HIGH)

## Speed and Acceleration Settings

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
```

## Special Position Constants

```cpp
// Specific positions for operation
const float WOOD_SUCTION_CHECK_POSITION = 0.3;  // inches - Position to check suction
const float TRANSFER_ARM_SIGNAL_POSITION = 7.2; // inches - Position to signal transfer arm (500ms pulse)
const float CUT_COMPLETE_POSITION = 8.5;        // inches - Full travel of cut motor
const float POSITION_MOTOR_RETRACT_DISTANCE = 0.1; // inches - Distance at which to retract position clamp
```

## Motor Configuration Implementation

```cpp
// Motor instances (DRIVER interface mode)
AccelStepper cutMotor(AccelStepper::DRIVER, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(AccelStepper::DRIVER, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

// Configure motors during setup
void configureMotors() {
  // Cut motor configuration
  cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
  cutMotor.setCurrentPosition(0);
  
  // Position motor configuration
  positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
  positionMotor.setCurrentPosition(0);
}

// Non-blocking motor run function (must be called in main loop)
void runMotors() {
  cutMotor.run();
  positionMotor.run();
}
```

## Position Conversion Functions

```cpp
// Convert inches to steps
float inchesToSteps(float inches, float stepsPerInch) {
  return inches * stepsPerInch;
}

// Convert steps to inches
float stepsToInches(long steps, float stepsPerInch) {
  return (float)steps / stepsPerInch;
}
```

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