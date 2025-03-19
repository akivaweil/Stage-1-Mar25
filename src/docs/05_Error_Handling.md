# Error Handling

This document describes the error handling mechanisms and recovery procedures for the wood cutting machine.

## Error Detection

### Safety Critical Errors (Immediate Stop)
- Safety sensor triggered (active HIGH)
- Motor stall detected
- Position limit exceeded
- Unexpected position switch activation

### Operational Errors
- Wood suction check failed (vacuum sensor active LOW when wood present)
- Homing sequence failed 
- Cutting operation timeout
- Unexpected system state

## Error State Transition

When an error is detected:

1. All motors must stop immediately
2. System transitions to ERROR_STATE
3. Error code and description may be displayed via Serial (only in ERROR_STATE)
4. Visual indicator (LED) should be activated
5. System awaits reset/restart

```cpp
// Example error state transition
void handleError(int errorCode) {
  // Stop all motors immediately
  cutMotor.stop();
  positionMotor.stop();
  
  // Disable all outputs (pneumatics, etc.)
  digitalWrite(CLAMP_SOLENOID_PIN, LOW);
  digitalWrite(VACUUM_SOLENOID_PIN, LOW);
  digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW);
  
  // Turn on error indicator
  digitalWrite(ERROR_LED_PIN, HIGH);
  
  // Store error code
  currentErrorCode = errorCode;
  
  // Transition to ERROR_STATE
  currentState = ERROR_STATE;
  
  // Now that we're in ERROR_STATE, we can print to Serial
  Serial.print("ERROR: ");
  Serial.println(errorNames[errorCode]);
}
```

## Error Codes

```cpp
// Error code definitions
enum ErrorCodes {
  ERROR_NONE = 0,
  ERROR_SAFETY_SENSOR = 1,
  ERROR_MOTOR_STALL = 2,
  ERROR_POSITION_LIMIT = 3,
  ERROR_UNEXPECTED_SWITCH = 4,
  ERROR_SUCTION_FAILED = 5,
  ERROR_HOMING_FAILED = 6,
  ERROR_OPERATION_TIMEOUT = 7,
  ERROR_INVALID_STATE = 8
};

// Error names for display
const char* errorNames[] = {
  "No Error",
  "Safety Sensor Triggered",
  "Motor Stall Detected",
  "Position Limit Exceeded",
  "Unexpected Switch Activation",
  "Wood Suction Failed",
  "Homing Sequence Failed",
  "Operation Timeout",
  "Invalid System State"
};
```

## Error Recovery Procedures

### Standard Recovery Process
1. Clear the error condition (physical reset may be required)
2. Press the reset button (active LOW)
3. System must perform a complete homing sequence before resuming normal operation
4. After successful homing, system returns to IDLE_STATE

```cpp
// Example error recovery function
void handleErrorRecovery() {
  // Check if reset button is pressed (active LOW)
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    // Check if the error condition is cleared
    if (errorConditionCleared()) {
      // Turn off error indicator
      digitalWrite(ERROR_LED_PIN, LOW);
      
      // Clear error code
      currentErrorCode = ERROR_NONE;
      
      // Transition to HOMING_STATE
      currentState = HOMING_STATE;
      
      // Reset all position data
      resetPositions();
      
      // Optional: Print recovery message
      Serial.println("Error cleared. Starting homing sequence.");
    }
    else {
      // Error condition still exists
      Serial.println("Cannot reset: Error condition still present.");
    }
  }
}

// Check if error condition is cleared
bool errorConditionCleared() {
  switch (currentErrorCode) {
    case ERROR_SAFETY_SENSOR:
      return digitalRead(SAFETY_SENSOR_PIN) == LOW; // Safety sensor not triggered (active HIGH)
      
    case ERROR_SUCTION_FAILED:
      // Check if vacuum is working or cycle was aborted
      return true; // Requires new cycle to test
      
    // Add checks for other error types
      
    default:
      return true; // For errors that don't require specific condition checks
  }
}
```

## Implementing Timeout Detection

Timeouts are implemented using non-blocking timing:

```cpp
// Example timeout implementation
unsigned long operationStartTime = 0;
const unsigned long OPERATION_TIMEOUT_MS = 30000; // 30 seconds timeout

// Check for operation timeout
void checkOperationTimeout() {
  if (operationStartTime > 0 && millis() - operationStartTime > OPERATION_TIMEOUT_MS) {
    handleError(ERROR_OPERATION_TIMEOUT);
  }
}

// Start operation timer
void startOperationTimer() {
  operationStartTime = millis();
}

// Reset operation timer
void resetOperationTimer() {
  operationStartTime = 0;
}
```

## Safety Checks

Safety checks must be performed continuously:

```cpp
// Safety check function called in every loop iteration
void performSafetyChecks() {
  // Check safety sensor (active HIGH)
  if (digitalRead(SAFETY_SENSOR_PIN) == HIGH) {
    handleError(ERROR_SAFETY_SENSOR);
    return;
  }
  
  // Check motor positions (if motors are enabled)
  if (motorsEnabled) {
    // Check for position limit exceeded
    if (stepsToInches(cutMotor.currentPosition(), CUT_MOTOR_STEPS_PER_INCH) > CUT_MOTOR_TRAVEL_DISTANCE + 0.1 ||
        stepsToInches(positionMotor.currentPosition(), POSITION_MOTOR_STEPS_PER_INCH) > POSITION_MOTOR_TRAVEL_DISTANCE + 0.1) {
      handleError(ERROR_POSITION_LIMIT);
      return;
    }
  }
  
  // Additional safety checks as needed
}
```

## Sensor Logic Levels for Error Detection
- WAS_WOOD_SUCTIONED_SENSOR: Active LOW (suction is proper when reading HIGH)
- CUT_MOTOR_POSITION_SWITCH: Active HIGH (switch triggered when reading HIGH)
- All inputs use consistent 15ms debounce time

## Error Types

### 1. WOOD_SUCTION_ERROR
- **Triggered when**: WAS_WOOD_SUCTIONED_SENSOR indicates failure (reads LOW) at exactly the 0.3-inch point in the cut
- **Visual indicator**: Red LED blinks in pattern of 3 quick flashes followed by a pause (3 flashes, 1-second pause, repeat)
- **Resolution**: Operator must check vacuum system and remove any debris or blockage
- **System behavior**: Immediately begins returning cut motor to position zero while maintaining clamp extension
- **Recovery**: 
  1. System waits until Cycle Switch is toggled OFF
  2. When Cycle Switch is turned ON again (edge detection), system returns to HOMING_STATE
  3. System performs the same safety check as during startup
  4. Normal operation resumes only after successful homing

### 2. CUT_MOTOR_HOME_ERROR
- **Triggered when**: Cut motor position switch does not read HIGH after motor completes return to home position
- **Visual indicator**: Red LED blinks in pattern of 2 quick flashes followed by a pause (2 flashes, 1-second pause, repeat)
- **Possible causes**: Mechanical obstruction, motor step loss, or switch failure
- **System behavior**: Attempted automatic recovery to reach position zero
- **Recovery sequence**:
  1. First attempt: System calls `moveAwayThenHomeCutMotor()` function to move motor 1 inch forward with reduced acceleration (1/10th), then attempts to rehome
  2. If first rehoming fails: System calls `moveAwayThenHomeCutMotor()` again for a second attempt
  3. If second rehoming fails: System waits for Cycle Switch toggle (OFF→ON) to return to HOMING_STATE
  4. Position clamp and wood secure clamp remain extended to maintain secure grip on wood

## Standardized Error Recovery Process

All error states use the same edge-detection method for cycle switch toggle:
```cpp
bool prevCycleSwitchState = false;  // Global variable to track previous state

// In error state handler:
bool currentCycleSwitchState = readCycleSwitch();
if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
  // Rising edge detected (OFF→ON toggle)
  currentError = NO_ERROR;
  enterState(HOMING_STATE);
}
prevCycleSwitchState = currentCycleSwitchState;
```

## Error Recovery Flow

1. When any error occurs, the system immediately transitions to the appropriate error state
2. The red LED begins blinking in the pattern specific to the error type
3. The system immediately begins returning motors to position zero at a safe speed
4. All clamps remain extended during error recovery (immediate operations, no delays)
5. System monitors for Cycle Switch toggle (OFF→ON transition)
6. After detecting toggle, system resets error and returns to HOMING_STATE
7. Normal operation resumes only after successful homing

## No Serial Communication
- No Serial.print statements are used for error reporting
- System relies solely on LED patterns to indicate error conditions

## Error Type Enumeration

```cpp
enum ErrorType {
  NO_ERROR,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR
};
```

## Error LED Pattern Timing

```cpp
// Error LED timing constants
const unsigned long RED_LED_ERROR_FLASH_ON = 200;   // ms for error LED flash on time
const unsigned long RED_LED_ERROR_FLASH_OFF = 200;  // ms for error LED flash off time
const unsigned long RED_LED_ERROR_PAUSE = 1000;     // ms for pause after error pattern
```

## CUT_MOTOR_HOME_ERROR Recovery Implementation

```cpp
void handleCutMotorHomeErrorState() {
  switch (subState) {
    case 0:  // First recovery attempt
      // Update error LED pattern
      updateRedLEDErrorPattern(CUT_MOTOR_HOME_ERROR);
      
      // Keep clamps extended for safety
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // First attempt - use moveAwayThenHomeCutMotor which implements
      // reduced acceleration for the move-away portion
      moveAwayThenHomeCutMotor();
      
      // Check if homing was successful
      if (readCutMotorPositionSwitch()) {
        // Success, reset error
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      // First attempt failed, increment counter and try again
      homingAttemptCount++;
      subState = 1;
      break;
      
    case 1:  // Second recovery attempt
      // Second attempt - use moveAwayThenHomeCutMotor again
      moveAwayThenHomeCutMotor();
      
      // Check if second attempt was successful
      if (readCutMotorPositionSwitch()) {
        // Success, reset error
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      // Second attempt failed, move to waiting for cycle switch toggle
      homingAttemptCount++;
      subState = 2;
      break;
      
    case 2:  // Waiting for cycle switch toggle
      // Update error LED pattern continuously
      updateRedLEDErrorPattern(CUT_MOTOR_HOME_ERROR);
      
      // Keep clamps extended for safety
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // Monitor cycle switch for toggle (OFF→ON)
      bool currentCycleSwitchState = readCycleSwitch();
      if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
        // Cycle switch has been toggled OFF→ON
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
      }
      prevCycleSwitchState = currentCycleSwitchState;
      break;
  }
} 