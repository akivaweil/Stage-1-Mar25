# Error Handling

The system enters the ERROR_STATE due to specific detected conditions. Error handling is critical for safe operation of the wood cutting machine.

## Error Types

### 1. WOOD_SUCTION_ERROR
- **Triggered when**: WAS_WOOD_SUCTIONED_SENSOR indicates failure (Active LOW) at exactly the 0.3-inch point in the cut
- **Visual indicator**: Red LED blinks in pattern of 3 quick flashes followed by a pause (3 flashes, 1-second pause, repeat)
- **Resolution**: Operator must check vacuum system and remove any debris or blockage
- **System behavior**: Immediately begins returning cut motor to position zero while maintaining clamp extension
- **Recovery**: 
  1. System waits until Cycle Switch is toggled OFF
  2. When Cycle Switch is turned ON again, system returns to HOMING_STATE
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
  3. If second rehoming fails: System stops completely and requires a full power reset
  4. During this error state, the system will not respond to any switch inputs
  5. Position clamp and wood secure clamp remain extended to maintain secure grip on wood

## Error Recovery Flow

1. When any error occurs, the system immediately transitions to the appropriate error state
2. The red LED begins blinking in the pattern specific to the error type
3. The system immediately begins returning motors to position zero at a safe speed
4. The system waits for operator intervention
5. For WOOD_SUCTION_ERROR_STATE: Toggle Cycle Switch OFF then ON
6. For CUT_MOTOR_HOME_ERROR_STATE after second failure: Perform a full power reset
7. After recovery action, system returns to HOMING_STATE to re-establish reference positions
8. Normal operation resumes only after successful homing

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
      
      // Second attempt failed, system must be powered off
      homingAttemptCount++;
      subState = 2;
      break;
      
    case 2:  // Max attempts reached - system locked
      // Update error LED pattern continuously
      updateRedLEDErrorPattern(CUT_MOTOR_HOME_ERROR);
      
      // System is locked until power cycle
      // Keep clamps extended for safety
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // No transition possible from this state except power cycle
      break;
  }
} 