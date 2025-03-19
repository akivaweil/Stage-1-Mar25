# Operation Logic

## Core Principles
- **Immediate Action**: Commands must execute immediately without unnecessary delays
- **Non-Blocking Operations**: All motor movements must be non-blocking
- **No Serial Printing During Operations**: Serial.print statements must NOT be used during any motor or operational functions
- **Simple Sequential Steps**: Operations should occur in a clear, sequential order

## State Machine Implementation

The system operates as a finite state machine with the following states:

- `IDLE_STATE`: System powered on but inactive, awaiting commands
- `HOMING_STATE`: System performing homing sequence on startup
- `CUTTING_STATE`: System performing cutting operation
- `ERROR_STATE`: System encountered an error and requires intervention
- `YESWOOD_STATE`: System detected wood is present after cutting
- `NOWOOD_STATE`: System detected no wood is present after cutting

The state machine must be implemented in a way that allows immediate transitions between states based on trigger conditions.

## Operational Workflow

### 1. Power-Up Sequence
- System transitions immediately to `HOMING_STATE` on startup (no delays)
- No Serial.print statements should be used during this transition
- All motors must home before any operation is allowed

### 2. Typical Operation Sequence
```cpp
// Example of a non-blocking operation sequence
void loop() {
  switch (currentState) {
    case IDLE_STATE:
      // Check for start button press (active LOW)
      if (digitalRead(START_BUTTON_PIN) == LOW) {
        currentState = CUTTING_STATE;
        // Reset positions and prepare for cutting cycle
        prepareForCutting();
      }
      break;
      
    case CUTTING_STATE:
      // Execute cutting cycle steps in non-blocking way
      if (cutStateMachine()) {
        // Cutting cycle completed successfully
        // Transition handled within cutStateMachine() based on wood presence
      }
      break;
      
    case ERROR_STATE:
      // Handle error conditions
      handleErrors();
      break;
      
    case HOMING_STATE:
      // Execute homing sequence in non-blocking way
      if (homeStateMachine()) {
        // Homing completed successfully
        currentState = IDLE_STATE;
      }
      break;
      
    case YESWOOD_STATE:
      // Handle operations when wood is present
      if (handleYesWoodState()) {
        // YESWOOD state operations complete
        // Transition to IDLE_STATE handled within handleYesWoodState()
      }
      break;
      
    case NOWOOD_STATE:
      // Handle operations when no wood is present
      if (handleNoWoodState()) {
        // NOWOOD state operations complete
        // Transition to IDLE_STATE handled within handleNoWoodState()
      }
      break;
  }
  
  // Always run motors (non-blocking)
  runMotors();
  
  // Always check safety sensors (active HIGH when triggered)
  checkSafetySensors();
}
```

## Operational Notes

### Motor Control Requirements
- All movement commands must execute immediately without unnecessary delays
- Operations must be structured to avoid blocking the main loop
- Always call `runMotors()` in the main loop for non-blocking motor operation

### Signal Handling
- All input signals must be debounced appropriately
- Position switches are active HIGH (signal is HIGH when triggered)
- Button inputs are active LOW (signal is LOW when pressed)
- Output signals should be held for appropriate durations using non-blocking timing

### Error Recovery
- Error recovery must be clearly defined for each potential error condition
- After error recovery, system should return to IDLE_STATE
- All motors must home again after an error before resuming normal operation

### Safety Monitoring
- Safety sensors must be checked continuously in every loop iteration
- Any safety trigger must immediately stop all motors and transition to ERROR_STATE

## Diagnostic Output

Diagnostic Serial.print statements may be used ONLY during:

1. Initial setup
2. IDLE_STATE
3. ERROR_STATE
4. When a state change occurs and no motors are moving

```cpp
// Example of appropriate diagnostic output
void changeState(int newState) {
  // Only print when changing states and no motors are moving
  if (cutMotor.isRunning() == false && positionMotor.isRunning() == false) {
    Serial.print("State changing from ");
    Serial.print(stateNames[currentState]);
    Serial.print(" to ");
    Serial.println(stateNames[newState]);
  }
  
  currentState = newState;
}
```

## Implementation Requirements

1. All operations must be implemented using non-blocking approaches
2. Input signals must be properly debounced
3. No Serial.print statements during any motor movements or operational functions
4. Maintain clear state transitions with appropriate conditions
5. Always check safety conditions in every loop iteration

## YESWOOD and NOWOOD States

These states handle the system behavior after a cutting cycle based on whether wood is detected or not.

### YESWOOD_STATE
When wood is detected at the end of a cut, the system:
1. Keeps the wood secure clamp extended
2. Retracts the position clamp
3. Moves the position motor to the next position
4. Transitions to IDLE_STATE when positioning is complete

```cpp
void initializeYesWood() {
  // Keep wood secure clamp extended
  extendWoodSecureClamp();
  
  // Retract position clamp to allow positioning
  retractPositionClamp();
  
  // Move position motor to next position
  positionMotor.moveTo(inchesToSteps(POSITION_MOTOR_TRAVEL_DISTANCE, POSITION_MOTOR_STEPS_PER_INCH));
}

bool handleYesWoodState() {
  // Check if positioning is complete
  if (positionMotor.isRunning() == false) {
    // Transition to IDLE_STATE
    changeState(IDLE_STATE);
    return true;
  }
  return false; // Still in progress
}
```

### NOWOOD_STATE
When no wood is detected at the end of a cut, the system:
1. Retracts all clamps
2. Returns the position motor to home position
3. Transitions to IDLE_STATE when homing is complete

```cpp
void initializeNoWood() {
  // Retract all clamps
  retractPositionClamp();
  retractWoodSecureClamp();
  
  // Return position motor to home
  positionMotor.moveTo(0);
}

bool handleNoWoodState() {
  // Check if homing is complete
  if (positionMotor.isRunning() == false) {
    // Transition to IDLE_STATE
    changeState(IDLE_STATE);
    return true;
  }
  return false; // Still in progress
}
```

## General Operation Flow

- System performs homing sequence immediately after startup without delay
- In IDLE_STATE, system monitors Cycle Switch and Reload Switch inputs
- RELOAD_STATE is only accessible from IDLE_STATE by activating the Reload Switch
- Both clamps remain retracted during RELOAD_STATE to facilitate wood handling (immediate operations, no delays)
- System must return to IDLE_STATE from RELOAD before starting a cutting cycle
- This safety feature prevents accidental cycle activation during material loading
- WAS_WOOD_SUCTIONED_SENSOR is checked exactly once at 0.3 inches into the cut (active LOW - suction proper when HIGH)
- Transfer Arm signal is sent precisely at 7.2 inches of cut motor travel (non-blocking 500ms HIGH pulse)
- YESorNO_WOOD_SENSOR is checked at the very end of the cutting movement (active LOW - wood present when LOW)
- Machine continues cycling if Cycle Switch remains activated until NOWOOD_STATE is completed
- NOWOOD_STATE still allows one final cut to be performed before executing NOWOOD_STATE movements
- Safety check prevents operation if Cycle Switch is ON during startup (requires toggle OFF→ON)
- Error detection continuously monitors for suction and homing failures
- WAS_WOOD_SUCTIONED_SENSOR monitoring detects improper cutting conditions early in the process
- YESorNO_WOOD_SENSOR determines if the machine should continue to the YESWOOD_STATE or NOWOOD_STATE

## Safety Features

- Homing sequence ensures accurate reference positions for reliable operation
- Switch debouncing (15ms) prevents false readings and improper activations
- Both clamps must be engaged before cutting begins
- WAS_WOOD_SUCTIONED_SENSOR monitoring detects improper cutting conditions early in the process
- Cycle Switch safety check prevents unexpected operation at startup
- RELOAD_STATE requires returning to IDLE_STATE before cutting operations can begin
- Clear LED indicators provide visual feedback of system status
- All errors require Cycle Switch toggle (OFF→ON) to reset

## Cycle Switch Toggle Requirement

```cpp
// Global flag
bool needCycleSwitchToggle = false;

// Check function during startup
bool performStartupSafetyCheck() {
  if (readCycleSwitch()) {
    needCycleSwitchToggle = true;
    return false;
  }
  needCycleSwitchToggle = false;
  return true;
}

// In handleIdleState, check flag before starting cycle
if (readCycleSwitch()) {
  if (needCycleSwitchToggle) {
    return;  // Don't start cycle yet
  }
  enterState(CUTTING_STATE);
}

// Clear flag when cycle switch is toggled OFF
if (needCycleSwitchToggle && !readCycleSwitch()) {
  needCycleSwitchToggle = false;
}
```

## Non-Blocking Operation

- System uses non-blocking delays via the Wait() function
- Continuous monitoring of switches occurs during all delay periods
- Motor operations execute without blocking the main control loop
- No Serial communication or printing in main operations

## Switch and Sensor Configuration

- All switches and sensors use 15ms debounce time
- All inputs configured with INPUT_PULLUP mode
- YESorNO_WOOD_SENSOR and WAS_WOOD_SUCTIONED_SENSOR are active LOW
- All other switches are active HIGH

## Timing Management

```cpp
// Non-blocking delay function
bool Wait(unsigned long delayTime, unsigned long* startTimePtr) {
  // First time entering this function
  if (*startTimePtr == 0) {
    *startTimePtr = millis();
    return false;
  }
  
  // Check if the delay time has elapsed
  if (millis() - *startTimePtr >= delayTime) {
    *startTimePtr = 0;  // Reset for next use
    return true;
  }
  
  return false;
}
```

## State Variables and Flags

```cpp
// State tracking variables
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;
unsigned long stateStartTime = 0;
int subState = 0;

// Operation flags
bool isHomingComplete = false;
bool isNoWoodCycleCompleted = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool hasWoodSensorBeenChecked = false;
bool isCycleInProgress = false;
bool needCycleSwitchToggle = false;  // Toggle requirement flag
bool prevCycleSwitchState = false;   // For edge detection
```

## Transfer Arm Signal Implementation

The system uses a non-blocking approach for the 500ms Transfer Arm signal:

```cpp
// Global variables for timing
unsigned long transferArmSignalStartTime = 0;
bool transferArmSignalActive = false;

// Main loop includes timing update
void loop() {
  updateAllSwitches();
  runMotors();
  updateTransferArmSignal();  // Handle timing without blocking
  updateStateMachine();
  updateLEDsForState(currentState);
}

// Timer update function
void updateTransferArmSignal() {
  if (transferArmSignalActive && (millis() - transferArmSignalStartTime >= 500)) {
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
    transferArmSignalActive = false;
  }
}
```