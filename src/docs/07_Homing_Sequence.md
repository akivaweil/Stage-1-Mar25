# Homing Sequence

This document describes the homing sequence for the wood cutting machine.

## Immediate Startup Transition

- System transitions from STARTUP_STATE to HOMING_STATE without delay
- No Serial.print statements should be used during motor movements
- Hardware initialization must be completed before homing begins

```cpp
void handleStartupState() {
  // Initialize system by setting all clamps to their default positions
  extendPositionClamp();
  extendWoodSecureClamp();

  // Set initial state for Transfer Arm signal
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);

  // Immediately transition to homing state
  enterState(HOMING_STATE);
}
```

## Homing Steps

The homing sequence must be implemented as a non-blocking state machine:

1. Move cut motor away from home switch if already activated (active HIGH)
2. Move position motor away from home switch if already activated (active HIGH) 
3. Move cut motor toward home position at homing speed
4. Stop cut motor immediately when homing switch activates (is HIGH)
5. Move position motor toward home position at homing speed
6. Stop position motor immediately when homing switch activates (is HIGH)
7. Set both motor positions to zero
8. Transition to READY_STATE

## Homing Implementation

```cpp
// Current homing state handled by subState variable within handleHomingState

// Homing state handler function
void handleHomingState() {
  // Implementation for the homing state
  // This is a state machine within a state machine
  // to handle the homing sequence non-blocking
  
  switch (subState) {
    case 0:  // Check if motors are already at home
      // Check if cut motor is already at home position (switch is active HIGH)
      if (readCutMotorHomingSwitch()) {
        subState = 1;  // Need to move cut motor away from home
      } else {
        subState = 2;  // Cut motor already away from home, check position motor
      }
      
      // Check if position motor is already at home position (switch is active HIGH)
      if (readPositionMotorHomingSwitch()) {
        if (subState == 2) {
          subState = 3;  // Need to move position motor away from home
        }
      } else {
        if (subState == 2) {
          subState = 4;  // Both motors away from home, proceed to homing cut motor
        }
      }
      break;
      
    case 1:  // Move cut motor away from home switch
      // Set motor speed for moving away
      cutMotor.setMaxSpeed(CUT_MOTOR_HOMING_SPEED);
      cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
      
      // Move motor slightly away from home switch (negative direction)
      if (!cutMotor.isRunning()) {
        cutMotor.move(-300);  // Move 300 steps away
      }
      
      // Check if motor has moved away from switch
      if (!readCutMotorHomingSwitch() || cutMotor.distanceToGo() == 0) {
        // Motor has moved away or completed movement
        cutMotor.stop();
        
        // Check if position motor also needs to move away
        if (readPositionMotorHomingSwitch()) {
          subState = 3;  // Move position motor away next
        } else {
          subState = 4;  // Proceed to homing cut motor
        }
      }
      break;
      
    case 2:  // Intermediate case - should never reach here
      // Safety case - proceed to next appropriate state
      subState = (readPositionMotorHomingSwitch()) ? 3 : 4;
      break;
      
    case 3:  // Move position motor away from home switch
      // Set motor speed for moving away
      positionMotor.setMaxSpeed(POSITION_MOTOR_HOMING_SPEED);
      positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
      
      // Move motor slightly away from home switch (negative direction)
      if (!positionMotor.isRunning()) {
        positionMotor.move(-300);  // Move 300 steps away
      }
      
      // Check if motor has moved away from switch
      if (!readPositionMotorHomingSwitch() || positionMotor.distanceToGo() == 0) {
        // Motor has moved away or completed movement
        positionMotor.stop();
        subState = 4;  // Proceed to homing cut motor
      }
      break;
      
    case 4:  // Home cut motor
      // Set motor speed for homing
      cutMotor.setMaxSpeed(CUT_MOTOR_HOMING_SPEED);
      cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
      
      // Start moving toward home if not already running
      if (!cutMotor.isRunning() && !readCutMotorHomingSwitch()) {
        cutMotor.move(15000);  // Move enough steps to reach home
      }
      
      // Check if home position reached
      if (readCutMotorHomingSwitch()) {
        // Stop immediately when home position reached
        cutMotor.stop();
        
        // Set current position as zero
        cutMotor.setCurrentPosition(0);
        
        // Cut motor is now homed
        isCutMotorHomed = true;
        
        // Proceed to home position motor
        subState = 5;
      } else if (cutMotor.distanceToGo() == 0) {
        // Failed to find home within expected distance
        homingAttemptCount++;
        
        if (homingAttemptCount >= 3) {
          // Multiple attempts failed, go to homing error state
          enterState(CUT_MOTOR_HOME_ERROR_STATE);
          return;
        } else {
          // Try again from a different position
          subState = 1;  // Go back to moving away first
        }
      }
      break;
      
    case 5:  // Home position motor
      // Set motor speed for homing
      positionMotor.setMaxSpeed(POSITION_MOTOR_HOMING_SPEED);
      positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
      
      // Start moving toward home if not already running
      if (!positionMotor.isRunning() && !readPositionMotorHomingSwitch()) {
        positionMotor.move(10000);  // Move enough steps to reach home
      }
      
      // Check if home position reached
      if (readPositionMotorHomingSwitch()) {
        // Stop immediately when home position reached
        positionMotor.stop();
        
        // Set current position as zero
        positionMotor.setCurrentPosition(0);
        
        // Position motor is now homed
        isPositionMotorHomed = true;
        
        // Both motors are now homed
        isHomingComplete = true;
        
        // Transition to READY_STATE
        enterState(READY_STATE);
      } else if (positionMotor.distanceToGo() == 0) {
        // Failed to find home within expected distance
        homingAttemptCount++;
        
        if (homingAttemptCount >= 3) {
          // Multiple attempts failed, go to error state
          enterState(ERROR_STATE);
          return;
        } else {
          // Try again from a different position
          subState = 3;  // Go back to moving away first
        }
      }
      break;
  }
}
```

## Homing Error Handling

If homing fails, the system enters either CUT_MOTOR_HOME_ERROR_STATE or ERROR_STATE depending on which motor failed to home:

```cpp
void handleCutMotorHomeErrorState() {
  // Keep updating the error LED pattern
  updateRedLEDErrorPattern(CUT_MOTOR_HOME_ERROR);
  
  // Keep clamps extended for safety
  extendPositionClamp();
  extendWoodSecureClamp();
  
  switch (subState) {
    case 0:  // First recovery attempt
      // First attempt - move away then try to home the cut motor
      moveAwayThenHomeCutMotor();
      
      // Check if homing was successful
      if (readCutMotorHomingSwitch()) {
        // Success, reset error and move to homing state
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      // First attempt failed, increment counter and move to next state
      homingAttemptCount++;
      subState = 1;
      break;
      
    case 1:  // Second recovery attempt
      // Second attempt - move away then try to home the cut motor again
      moveAwayThenHomeCutMotor();
      
      // Check if homing was successful
      if (readCutMotorHomingSwitch()) {
        // Success, reset error and move to homing state
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      // Second attempt failed, move to locked state
      homingAttemptCount++;
      subState = 2;
      break;
      
    case 2:  // Locked state - requires cycle switch toggle
      // Monitor cycle switch for toggle (OFF then ON)
      bool currentCycleSwitchState = readCycleSwitch();
      
      // Check for a complete toggle cycle (OFF then ON)
      if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
        // Cycle switch has been toggled off and then on again
        // Reset error and go to homing state for one more attempt
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
      }
      
      // Update the previous cycle switch state
      prevCycleSwitchState = currentCycleSwitchState;
      break;
  }
}
```

## Move Away Then Home Function

This helper function is used during homing and error recovery:

```cpp
void moveAwayThenHomeCutMotor() {
  static unsigned long startTime = 0;
  
  // First time entering this function or after completion
  if (startTime == 0) {
    // Move away from home position (negative direction)
    cutMotor.setMaxSpeed(CUT_MOTOR_HOMING_SPEED);
    cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
    cutMotor.move(-500);  // Move 500 steps away
    startTime = millis();
    return;
  }
  
  // Check if move away is complete
  if (!cutMotor.isRunning() || (millis() - startTime > 2000)) {
    // Stop motor explicitly
    cutMotor.stop();
    
    // Now move toward home position (positive direction)
    cutMotor.move(2000);  // Move 2000 steps toward home
    startTime = 0;  // Reset for next call
  }
}
```

## Verifying Homing Success

After homing is complete, both motors should be at their home positions and position values set to zero:

```cpp
bool isHomingComplete() {
  return isCutMotorHomed && isPositionMotorHomed;
}

bool isCutMotorHomed() {
  return cutMotor.currentPosition() == 0 && readCutMotorHomingSwitch();
}

bool isPositionMotorHomed() {
  return positionMotor.currentPosition() == 0 && readPositionMotorHomingSwitch();
}
```

## Transition to READY_STATE

Once homing is successfully completed, the system transitions to READY_STATE:

```cpp
// From within handleHomingState, final subState
if (isHomingComplete) {
  // Both motors are now homed successfully
  enterState(READY_STATE);
}
```

In READY_STATE, the system is prepared to begin the cutting cycle when the cycle switch is activated.

## Homing Switch Detection

- Homing switches are active HIGH (signal is HIGH when triggered)
- All inputs use 15ms debounce time for reliable reading
- Motors must stop immediately when switches activate

## Safety Considerations

1. **Slow Homing Speed**: Homing should be performed at a safe, slow speed to prevent damage
2. **Motor Stall Detection**: The system should detect if motors stall during homing
3. **Timeout Monitoring**: Homing should fail if it takes too long (timeout)
4. **Safe Movement Direction**: Motors should move in directions that won't damage the machine

## Homing Timing

- Homing should be performed without unnecessary delays
- The system should transition immediately to homing on startup
- After successful homing, the system should be ready for operation

## Homing Failure Handling

If homing fails:

1. Motors should stop immediately
2. System should transition to ERROR_STATE
3. Appropriate error code should be set
4. User must reset the system to attempt homing again

## Post-Homing Actions

After successful homing:

1. Motor positions are set to zero (reference point)
2. Motor speeds are reset to normal operation values
3. System transitions to READY_STATE
4. System is ready to accept commands

## Important Notes

- Successful homing is required before any cutting operations
- No Serial.print statements should be used during motor movements
- The homing sequence must be implemented as a non-blocking state machine
- Safety checks must remain active throughout the homing process

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
  if (readCutMotorHomingSwitch()) {
    moveAwayThenHomeCutMotor();
  } else {
    homeCutMotor();
  }
  
  if (readPositionMotorHomingSwitch()) {
    moveAwayThenHomePositionMotor();
  } else {
    homePositionMotor();
  }
  
  // Move position motor to operating position
  movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE);
  
  // Homing complete
  isHomingComplete = true;
}

// Dedicated function for moving away then homing with reduced acceleration
void moveAwayThenHomeCutMotor() {
  // Save original acceleration
  float originalAcceleration = cutMotor.getAcceleration();
  
  // Set reduced acceleration (1/10th of normal)
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION / 10.0);
  
  // Move cut motor away from switch by 1 inch
  moveCutMotorToPosition(-1.0);
  while (!isMotorInPosition(cutMotor, inchesToSteps(-1.0, CUT_MOTOR_STEPS_PER_INCH))) {
    runMotors();
  }
  
  // Restore original acceleration
  cutMotor.setAcceleration(originalAcceleration);
  
  // Now perform normal homing
  homeCutMotor();
}

// Dedicated function for moving away then homing the position motor with reduced acceleration
void moveAwayThenHomePositionMotor() {
  // Save original acceleration
  float originalAcceleration = positionMotor.getAcceleration();
  
  // Set reduced acceleration (1/10th of normal)
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION / 10.0);
  
  // Move position motor away from switch by 1 inch
  movePositionMotorToPosition(-1.0);
  while (!isMotorInPosition(positionMotor, inchesToSteps(-1.0, POSITION_MOTOR_STEPS_PER_INCH))) {
    runMotors();
  }
  
  // Restore original acceleration
  positionMotor.setAcceleration(originalAcceleration);
  
  // Now perform normal homing
  homePositionMotor();
}

void homeCutMotor() {
  // Set direction toward home switch
  cutMotor.setSpeed(-CUT_MOTOR_HOMING_SPEED);
  
  // Move until switch is triggered (active HIGH)
  while (!readCutMotorHomingSwitch()) {
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
  
  // Move until switch is triggered (active HIGH)
  while (!readPositionMotorHomingSwitch()) {
    positionMotor.runSpeed();
  }
  
  // Stop motor and set position to 0
  positionMotor.stop();
  positionMotor.setCurrentPosition(0);
  isPositionMotorHomed = true;
}
``` 