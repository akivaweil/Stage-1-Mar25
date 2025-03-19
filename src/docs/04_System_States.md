# System States

The wood cutting machine operates as a finite state machine with well-defined states and transitions.

## State Definitions

### STARTUP_STATE (0)
- Initial state when system is powered on
- Performs basic initialization tasks
- Immediately transitions to HOMING_STATE
- Should be brief with no delays

### HOMING_STATE (1)
- System is performing a homing sequence
- Motors moving to find home position (reference point)
- No Serial output should occur during motor movements
- Transitions to READY_STATE upon successful completion
- Transitions to ERROR_STATE if homing fails

### READY_STATE (2)
- System is powered on and ready for operation (formerly IDLE_STATE)
- All motors are stationary at home position
- Awaiting cycle switch activation (active HIGH)
- No pneumatic actuators are engaged
- Green LED is illuminated in this state

### RELOAD_STATE (3)
- System is in reload mode after reload switch is activated
- Clamps are retracted to allow wood loading
- Blue LED is illuminated in this state
- Returns to READY_STATE when reload switch is deactivated

### CUTTING_STATE (4)
- System is performing the cutting operation cycle
- Motors are moving through defined positions
- No Serial output should occur during this state
- Pneumatic actuators engage/disengage as needed
- Yellow LED is illuminated in this state
- Transitions to YESWOOD_STATE or NOWOOD_STATE upon completion

### YESWOOD_STATE (5)
- System has detected wood is present after cutting
- Prepares for the next cut cycle
- Position motor moves to final position for next cut
- Wood secure clamp remains extended
- Transitions to READY_STATE upon completion

### NOWOOD_STATE (6)
- System has detected no wood present after cutting
- Prepares to end the cutting sequence
- Retracts clamps and returns to home position
- Transitions to READY_STATE upon completion

### ERROR_STATE (7)
- General error state for handling various errors
- All motors are stopped
- All pneumatic actuators are in safe position (extended)
- Error code is stored and may be displayed via LEDs
- Awaiting cycle switch toggle (OFF then ON)
- Transitions to HOMING_STATE after successful reset

### WOOD_SUCTION_ERROR_STATE (8)
- Specialized error state for wood suction failures
- Motors return to home position
- Clamps remain extended for safety
- Red LED uses a specific blink pattern for this error
- Awaiting cycle switch toggle (OFF then ON)
- Transitions to HOMING_STATE after successful reset

### CUT_MOTOR_HOME_ERROR_STATE (9)
- Specialized error state for cut motor homing failures
- System attempts recovery with up to two additional homing attempts
- Clamps remain extended for safety
- Red LED uses a specific blink pattern for this error
- If recovery fails, awaits cycle switch toggle (OFF then ON)
- Transitions to HOMING_STATE after successful reset

## State Transition Rules

1. **Immediate Transitions**: State transitions must execute immediately without delays
2. **Motor Priorities**: When transitioning to a new state, motor operations take precedence over Serial output
3. **Transition Criteria**: Each state transition has specific criteria that must be met

```cpp
// State definitions
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

// Error type definitions
enum ErrorType {
  NO_ERROR,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR
};

// Current system state and error
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;
```

## State Transition Function

```cpp
// State transition function
void enterState(State newState) {
  // Exit actions for current state
  switch (currentState) {
    case CUTTING_STATE:
      // Ensure transfer arm signal is off when exiting cutting state
      signalTransferArm(LOW);
      break;
      
    case ERROR_STATE:
    case WOOD_SUCTION_ERROR_STATE:
    case CUT_MOTOR_HOME_ERROR_STATE:
      // Turn off error LED when exiting error states
      digitalWrite(RED_LED_PIN, LOW);
      break;
      
    // Handle other state exits as needed
  }
  
  // Update state
  currentState = newState;
  stateStartTime = millis();
  subState = 0;
  
  // Entry actions for new state
  switch (newState) {
    case HOMING_STATE:
      // Start blue LED blinking for homing
      break;
      
    case READY_STATE:
      // Turn on green LED
      digitalWrite(GREEN_LED_PIN, HIGH);
      break;
      
    case RELOAD_STATE:
      // Turn on blue LED for reload
      digitalWrite(BLUE_LED_PIN, HIGH);
      break;
      
    // Handle other state entries as needed
  }
}
```

## System Startup Behavior

On system startup or reset:
1. System immediately enters STARTUP_STATE
2. Performs basic initialization with no delays
3. Transitions immediately to HOMING_STATE
4. Homing sequence begins automatically
5. After successful homing, system transitions to READY_STATE
6. If homing fails, system transitions to appropriate error state

```cpp
void setup() {
  // Initialize all pins
  initializePins();
  
  // Initialize switch debouncing
  initializeDebounce();
  
  // Configure motors
  configureMotors();
  
  // Check cycle switch at startup
  performStartupSafetyCheck();
  
  // Initial state
  currentState = STARTUP_STATE;
}
```

## LED Status Indicators

Each state has a distinct LED pattern:
- STARTUP_STATE: Blue LED on
- HOMING_STATE: Blue LED blinking
- READY_STATE: Green LED on
- RELOAD_STATE: Blue LED on
- CUTTING_STATE: Yellow LED on
- YESWOOD_STATE: Green and Yellow LEDs on
- NOWOOD_STATE: Blue and Green LEDs on
- ERROR_STATE: Red LED pattern (based on error type)
- WOOD_SUCTION_ERROR_STATE: Specific Red LED pattern
- CUT_MOTOR_HOME_ERROR_STATE: Specific Red LED pattern
