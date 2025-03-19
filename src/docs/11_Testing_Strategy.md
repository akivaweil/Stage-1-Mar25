# Testing Strategy

This document outlines the approach for testing the wood cutting machine control system to ensure reliable and safe operation.

## Unit Test Plan

### 1. Switch and Sensor Tests

- **Test objective**: Verify all switches and sensors read correct values with proper debouncing
- **Test procedure**:
  1. Test each switch individually with debouncing
  2. Verify position switches work reliably at their physical locations
  3. Test YESorNO_WOOD_SENSOR with and without wood present
  4. Test WAS_WOOD_SUCTIONED_SENSOR with and without proper suction
- **Expected results**: All sensor readings are stable, debounced, and accurately reflect physical states

### 2. Motor Tests

- **Test objective**: Verify motors move to the correct positions with proper acceleration
- **Test procedure**:
  1. Test homing sequence for both motors
  2. Verify motors stop immediately when position switches are triggered
  3. Test movement to specific positions (e.g., 1 inch, full travel)
  4. Verify speed and acceleration parameters are appropriate
- **Expected results**: Motors move smoothly to target positions and respond correctly to position switches

### 3. Clamp Tests

- **Test objective**: Verify clamps extend and retract correctly
- **Test procedure**:
  1. Test position clamp extension and retraction
  2. Test wood secure clamp extension and retraction
  3. Verify proper sequencing during reload and cutting operations
- **Expected results**: Clamps operate correctly and in the proper sequence for each operation

### 4. Error Tests

- **Test objective**: Verify error detection and recovery mechanisms
- **Test procedure**:
  1. Simulate wood suction error by forcing WAS_WOOD_SUCTIONED_SENSOR LOW at 0.3 inches
  2. Test cut motor home error recovery (first and second attempts)
  3. Verify error LED patterns match documentation
  4. Test error recovery through cycle switch toggling
- **Expected results**: System detects errors, displays correct indicators, and recovers appropriately

### 5. Full Cycle Tests

- **Test objective**: Verify complete cutting cycle in various conditions
- **Test procedure**:
  1. Test YESWOOD sequence with wood present
  2. Test NOWOOD sequence without wood
  3. Test continuous cycling with cycle switch held ON
  4. Verify transitions between states match the state transition table
- **Expected results**: Complete cutting cycles execute correctly in all scenarios

## Mock Hardware Testing

During development without hardware, use a mock hardware testing approach:

```cpp
// Mock hardware configuration
#define MOCK_HARDWARE  // Comment this out for actual hardware testing

// Mock hardware readings
bool mockSwitchValues[6] = {false, false, false, false, false, true};  // Default values
float mockMotorPositions[2] = {0.0, 0.0};  // [cutMotor, positionMotor] in inches

// Example mock function
bool readCutMotorPositionSwitch() {
  #ifdef MOCK_HARDWARE
    return mockSwitchValues[0];
  #else
    return cutMotorPositionSwitch.read();
  #endif
}

// Mock motor position
float getMotorPosition(int motorIndex) {
  #ifdef MOCK_HARDWARE
    return mockMotorPositions[motorIndex];
  #else
    if (motorIndex == 0) {
      return stepsToInches(cutMotor.currentPosition(), CUT_MOTOR_STEPS_PER_INCH);
    } else {
      return stepsToInches(positionMotor.currentPosition(), POSITION_MOTOR_STEPS_PER_INCH);
    }
  #endif
}

// Update mock motor position (simulate movement)
void updateMockMotorPosition(int motorIndex, float targetPosition) {
  #ifdef MOCK_HARDWARE
    // Simple simulation of movement - in real implementation, add gradual movement
    mockMotorPositions[motorIndex] = targetPosition;
    
    // Simulate triggering position switches when near zero
    if (motorIndex == 0 && mockMotorPositions[0] <= 0.05) {
      mockSwitchValues[0] = true;  // Cut motor position switch
    } else if (motorIndex == 1 && mockMotorPositions[1] <= 0.05) {
      mockSwitchValues[1] = true;  // Position motor position switch
    }
  #endif
}
```

## Integration Testing

### Sensor to Motor Response Tests

- **Test objective**: Verify motors respond correctly to sensor inputs
- **Test procedure**:
  1. Test homing sequence with position switches
  2. Test cut motor stopping at end position
  3. Verify response to WAS_WOOD_SUCTIONED_SENSOR during cutting
- **Expected results**: Motors respond appropriately to all sensor inputs

### State Transition Tests

- **Test objective**: Verify all state transitions occur correctly
- **Test procedure**:
  1. Test each transition in the state transition table
  2. Verify entry and exit actions for each state
  3. Test illegal transitions (e.g., RELOAD_STATE to CUTTING_STATE)
- **Expected results**: State machine follows the documented state transition rules

## Safety Testing

### Error Recovery Tests

- **Test objective**: Verify system recovers safely from error conditions
- **Test procedure**:
  1. Test recovery from WOOD_SUCTION_ERROR_STATE
  2. Test recovery from CUT_MOTOR_HOME_ERROR_STATE (both successful and failed recovery)
  3. Verify motors move at safe speeds during error recovery
- **Expected results**: System recovers safely from all error conditions

### Startup Safety Tests

- **Test objective**: Verify safety checks during startup
- **Test procedure**:
  1. Test startup with cycle switch already ON
  2. Verify homing sequence completes before allowing operation
- **Expected results**: System prevents unsafe operation during startup

## Performance Testing

### Timing Tests

- **Test objective**: Verify operation completes within expected timeframes
- **Test procedure**:
  1. Measure time for complete cutting cycle
  2. Test non-blocking behavior during operations
  3. Verify sensor checks occur at exact positions
- **Expected results**: All operations complete within expected timeframes

### Long-Term Stability Test

- **Test objective**: Verify system stability over extended operation
- **Test procedure**:
  1. Run multiple consecutive cutting cycles (100+)
  2. Monitor for any drift in positioning or timing
- **Expected results**: System maintains accuracy and stability over extended operation 