# System States

The wood cutting machine operates using a state machine with the following distinct states:

## 1. STARTUP_STATE
- Initial state when system powers on
- Initializes all pins and system configurations
- Transitions directly to HOMING_STATE

## 2. HOMING_STATE
- Runs the homing sequence to establish reference positions for both motors
- Blue LED blinks at 2-second intervals during this process
- Transitions to READY_STATE when complete

## 3. READY_STATE
- System waits for user input
- Green LED illuminates to indicate ready status
- Monitors Cycle Switch and Reload Switch for user actions
- Moves to RELOAD_STATE when Reload Switch is activated
- Moves to CUTTING_STATE when Cycle Switch is activated

## 4. RELOAD_STATE
- Accessible only from the READY_STATE
- Retracts both clamps to allow wood loading/unloading
- Blue LED illuminates to indicate reload mode
- Monitors Reload Switch for deactivation
- Must return to READY_STATE when Reload Switch is released before any cutting operation can begin
- Cannot directly transition to CUTTING_STATE as a safety measure

## 5. CUTTING_STATE
- Performs the cutting operation
- Verifies both clamps are properly engaged
- Moves cut motor to the cutting position (8.5 inches)
- Checks WAS_WOOD_SUCTIONED_SENSOR exactly once at 0.3 inches into the cut
- Signals the Transfer Arm (TA) when cut motor reaches 7.2 inches (0.5 inches before end of travel)
- Checks YESorNO_WOOD_SENSOR at the very end of the cutting movement
- Transitions to YESWOOD_STATE or NOWOOD_STATE based on YESorNO_WOOD_SENSOR reading

## 6. YESWOOD_STATE
- Handles sequence when wood is present for further cutting
- Returns both motors to home position
- Moves position motor to final position for next cut
- Manages clamp engagement/retraction during the sequence
- Returns to READY_STATE when complete

## 7. NOWOOD_STATE
- Handles sequence when no wood remains for cutting
- Returns both motors to home position
- Sets the isNoWoodCycleCompleted flag
- Returns to READY_STATE when complete

## 8. ERROR_STATE
- Manages various error conditions
- Displays distinct LED patterns based on error type
- Error is reset by toggling the Cycle Switch OFF and then ON
- Returns to HOMING_STATE when reset
- Immediately begins returning motors to position zero (does not halt in place)
- Keeps all clamps extended (engaged) to maintain secure grip on wood
- Moves at a safe, controlled speed during error recovery

## 9. WOOD_SUCTION_ERROR_STATE
- Triggered when: WAS_WOOD_SUCTIONED_SENSOR indicates failure at 0.3-inch point in the cut
- Visual indicator: Red LED blinks in pattern of 3 quick flashes followed by a pause
- Returns motors to position zero while maintaining clamp extension

## 10. CUT_MOTOR_HOME_ERROR_STATE
- Triggered when: Cut motor position switch does not read HIGH after homing
- Visual indicator: Red LED blinks in pattern of 2 quick flashes followed by a pause
- Attempts automatic recovery to reach position zero

## State Enumeration

```cpp
enum State {
  STARTUP_STATE,
  HOMING_STATE,
  READY_STATE,
  RELOAD_STATE,
  CUTTING_STATE,
  YESWOOD_STATE,
  NOWOOD_STATE,
  ERROR_STATE,
  WOOD_SUCTION_ERROR_STATE,
  CUT_MOTOR_HOME_ERROR_STATE
};
```

## State Transition Table

| Current State | Condition | Next State |
|---------------|-----------|------------|
| STARTUP_STATE | After initialization | HOMING_STATE |
| HOMING_STATE | Homing complete | READY_STATE |
| READY_STATE | Reload Switch ON | RELOAD_STATE |
| READY_STATE | Cycle Switch ON | CUTTING_STATE |
| RELOAD_STATE | Reload Switch OFF | READY_STATE |
| CUTTING_STATE | Cut complete, wood present | YESWOOD_STATE |
| CUTTING_STATE | Cut complete, no wood | NOWOOD_STATE |
| CUTTING_STATE | WAS_WOOD_SUCTIONED_SENSOR LOW at 0.3" | WOOD_SUCTION_ERROR_STATE |
| YESWOOD_STATE | After processing | READY_STATE |
| NOWOOD_STATE | After processing | READY_STATE |
| ERROR_STATE | Cycle Switch toggle | HOMING_STATE |
| WOOD_SUCTION_ERROR_STATE | Cycle Switch toggle | HOMING_STATE |
| CUT_MOTOR_HOME_ERROR_STATE | After 2 failed attempts | Requires power reset |
| CUT_MOTOR_HOME_ERROR_STATE | Successful recovery | HOMING_STATE | 