# Homing Sequence

The homing sequence establishes reference positions for both motors, which is essential for accurate and reliable operation.

## Homing Steps

1. If position switches are already triggered, motors move away by 1 inch first
2. Motors move toward their respective home position switches at homing speed
3. Motors stop immediately when switches are activated and positions are set to 0
4. Position motor explicitly returns to zero before moving to operating position (3.45 inches)
5. System is now homed and ready for operation

## Homing Speeds

```cpp
// Homing speeds
const float CUT_MOTOR_HOMING_SPEED = 300.0;  // steps/sec
const float POSITION_MOTOR_HOMING_SPEED = 2000.0;  // steps/sec
```

## Implementation Details

### Homing Functions

```cpp
void runHomingSequence() {
  // Set blue LED blinking pattern
  updateBlueLEDBlinking();
  
  // First check if switches are already triggered
  if (readCutMotorPositionSwitch()) {
    // Move cut motor away from switch by 1 inch
    moveCutMotorToPosition(-1.0);
    while (!isMotorInPosition(cutMotor, inchesToSteps(-1.0, CUT_MOTOR_STEPS_PER_INCH))) {
      runMotors();
    }
  }
  
  if (readPositionMotorPositionSwitch()) {
    // Move position motor away from switch by 1 inch
    movePositionMotorToPosition(-1.0);
    while (!isMotorInPosition(positionMotor, inchesToSteps(-1.0, POSITION_MOTOR_STEPS_PER_INCH))) {
      runMotors();
    }
  }
  
  // Home cut motor
  homeCutMotor();
  
  // Home position motor
  homePositionMotor();
  
  // Move position motor to operating position
  movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE);
  
  // Homing complete
  isHomingComplete = true;
}

void homeCutMotor() {
  // Set direction toward home switch
  cutMotor.setSpeed(-CUT_MOTOR_HOMING_SPEED);
  
  // Move until switch is triggered
  while (!readCutMotorPositionSwitch()) {
    cutMotor.runSpeed();
  }
  
  // Stop motor and set position to 0
  cutMotor.stop();
  cutMotor.setCurrentPosition(0);
  isCutMotorHomed = true;
}

void homePositionMotor() {
  // Set direction toward home switch
  positionMotor.setSpeed(-POSITION_MOTOR_HOMING_SPEED);
  
  // Move until switch is triggered
  while (!readPositionMotorPositionSwitch()) {
    positionMotor.runSpeed();
  }
  
  // Stop motor and set position to 0
  positionMotor.stop();
  positionMotor.setCurrentPosition(0);
  isPositionMotorHomed = true;
}
```

## Safety Considerations

- Homing must be completed successfully before any cutting operations
- If a home switch fails, the system cannot operate safely and must enter an error state
- The blue LED blinks during homing to indicate the system is not ready for operation
- The system verifies that both motors have reached their home positions correctly 