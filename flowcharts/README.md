# ESP32-S3 Stage 1 State Machine Flowchart

This flowchart visualizes the state machine implemented in the ESP32-S3 Stage 1 controller. 

## State Descriptions

- **STARTUP** - Initial state when the system powers on. Sets clamps and immediately transitions to HOMING.
- **HOMING** - Calibrates motors to their home positions.
- **READY** - System is ready and waiting for cycle switch to initiate cutting.
- **RELOAD** - Waiting for reload switch to be released after wood loading.
- **CUTTING** - Executing the cutting sequence with multiple substates.
- **YESWOOD** - Wood was detected after cutting; can either continue to next cut in continuous operation or prepare for a new cycle.
- **NOWOOD** - No wood detected after cutting; returns system to home.
- **ERROR** - General error state.
- **WOOD_SUCTION_ERROR** - Error specific to wood suction failure.
- **CUT_MOTOR_HOME_ERROR** - Error specific to cut motor homing failure.

## Transition Conditions

- **STARTUP → HOMING**: Immediate after startup
- **HOMING → READY**: Successful completion of homing sequence
- **HOMING → ERROR**: Position motor fails to home after 3 attempts
- **HOMING → CUT_MOTOR_HOME_ERROR**: Cut motor fails to home after 3 attempts
- **READY → RELOAD**: Reload switch is activated
- **READY → CUTTING**: Cycle switch is activated (and toggle not required)
- **RELOAD → READY**: Reload switch is deactivated
- **CUTTING → YESWOOD**: Wood is detected at end of cutting cycle
- **CUTTING → NOWOOD**: No wood is detected at end of cutting cycle
- **CUTTING → WOOD_SUCTION_ERROR**: Suction check fails during cutting
- **YESWOOD → CUTTING**: Continuous operation mode - system continues cutting while wood is available
- **YESWOOD → READY**: Operation complete or reload requested, ready for next operation
- **YESWOOD → CUT_MOTOR_HOME_ERROR**: Cut motor fails to return to home position
- **NOWOOD → READY**: Motors return to home, ready for next operation
- **Error states → HOMING**: Cycle switch is toggled, resetting the system

## LED Patterns

- **READY**: Green LED on
- **RELOAD**: Blue LED on
- **CUTTING**: Yellow LED on
- **YESWOOD**: Green + Yellow LEDs on
- **NOWOOD**: Blue + Green LEDs on
- **ERROR**: Red LED on
- **WOOD_SUCTION_ERROR**: Fast blinking red LED
- **CUT_MOTOR_HOME_ERROR**: Double pulse red LED pattern

## Notes

This flowchart shows the main state transitions but doesn't capture the substates within many states (particularly CUTTING and HOMING) which implement non-blocking sequences of operations.

The continuous operation mode (YESWOOD → CUTTING loop) allows the system to continue processing wood without returning to the READY state, improving efficiency when multiple pieces need to be cut. 