# State Machine Implementation

## State Machine Overview

The wood cutting machine operates as a finite state machine with 10 distinct states. Each state has well-defined entry and exit conditions, with specific operations performed within each state.

## State Definitions

1. **STARTUP_STATE**: Initial state on power-up, performs basic initialization
2. **HOMING_STATE**: Finds reference positions for motors
3. **READY_STATE**: System ready for operation, awaiting cycle start
4. **RELOAD_STATE**: System in reload mode, allows wood loading
5. **CUTTING_STATE**: Performing cutting operation
6. **YESWOOD_STATE**: Wood detected after cutting, positioning for next cut
7. **NOWOOD_STATE**: No wood detected after cutting, returning to home
8. **ERROR_STATE**: General error handling state
9. **WOOD_SUCTION_ERROR_STATE**: Specialized error handling for suction failures
10. **CUT_MOTOR_HOME_ERROR_STATE**: Specialized error handling for homing failures

## State Machine Implementation

```cpp
void updateStateMachine() {
  switch (currentState) {
    case STARTUP_STATE:
      handleStartupState();
      break;
      
    case HOMING_STATE:
      handleHomingState();
      break;
      
    case READY_STATE:
      handleReadyState();
      break;
      
    case RELOAD_STATE:
      handleReloadState();
      break;
      
    case CUTTING_STATE:
      handleCuttingState();
      break;
      
    case YESWOOD_STATE:
      handleYesWoodState();
      break;
      
    case NOWOOD_STATE:
      handleNoWoodState();
      break;
      
    case ERROR_STATE:
      handleErrorState();
      break;
      
    case WOOD_SUCTION_ERROR_STATE:
      handleWoodSuctionErrorState();
      break;
      
    case CUT_MOTOR_HOME_ERROR_STATE:
      handleCutMotorHomeErrorState();
      break;
  }
}
```

## State Transition Function

The state transition function handles cleanup of the previous state and initialization of the new state:

```cpp
void enterState(State newState) {
  // Exit actions for current state
  switch (currentState) {
    case STARTUP_STATE:
      // No exit actions
      break;
      
    case HOMING_STATE:
      // Exit homing - stop motors if needed
      break;
      
    case READY_STATE:
      // Exit ready state - turn off green LED
      digitalWrite(GREEN_LED_PIN, LOW);
      break;
      
    case RELOAD_STATE:
      // Exit reload state - turn off blue LED
      digitalWrite(BLUE_LED_PIN, LOW);
      break;
      
    case CUTTING_STATE:
      // Exit cutting - stop motors if needed
      signalTransferArm(LOW);  // Ensure transfer arm signal is off
      break;
      
    case YESWOOD_STATE:
      // No special exit actions
      break;
      
    case NOWOOD_STATE:
      // No special exit actions
      break;
      
    case ERROR_STATE:
      // Exit error state - turn off red LED
      digitalWrite(RED_LED_PIN, LOW);
      break;
      
    case WOOD_SUCTION_ERROR_STATE:
      // Exit wood suction error - turn off red LED
      digitalWrite(RED_LED_PIN, LOW);
      break;
      
    case CUT_MOTOR_HOME_ERROR_STATE:
      // Exit cut motor home error - turn off red LED
      digitalWrite(RED_LED_PIN, LOW);
      break;
  }
  
  // Update state
  currentState = newState;
  stateStartTime = millis();
  subState = 0;
  
  // Entry actions for new state
  switch (currentState) {
    case STARTUP_STATE:
      // Initialize for startup
      digitalWrite(BLUE_LED_PIN, HIGH);
      break;
      
    case HOMING_STATE:
      // Start homing sequence
      homingAttemptCount = 0;
      isHomingComplete = false;
      break;
      
    case READY_STATE:
      // Enter ready state - turn on green LED
      digitalWrite(GREEN_LED_PIN, HIGH);
      break;
      
    case RELOAD_STATE:
      // Enter reload state - turn on blue LED, retract clamps
      digitalWrite(BLUE_LED_PIN, HIGH);
      retractPositionClamp();
      retractWoodSecureClamp();
      break;
      
    case CUTTING_STATE:
      // Enter cutting state - turn on yellow LED
      digitalWrite(YELLOW_LED_PIN, HIGH);
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      break;
      
    case YESWOOD_STATE:
      // Enter yeswood state
      break;
      
    case NOWOOD_STATE:
      // Enter nowood state
      break;
      
    case ERROR_STATE:
      // Enter error state - setup red LED pattern
      currentError = NO_ERROR;
      break;
      
    case WOOD_SUCTION_ERROR_STATE:
      // Enter wood suction error state
      currentError = WOOD_SUCTION_ERROR;
      break;
      
    case CUT_MOTOR_HOME_ERROR_STATE:
      // Enter cut motor home error state
      currentError = CUT_MOTOR_HOME_ERROR;
      break;
  }
}
```

## Individual State Handlers

Each state has its own handler function that implements the state-specific logic:

### Startup State

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

### Homing State

```cpp
void handleHomingState() {
  // Implementation for the homing state
  // This will be a state machine within a state machine
  // to handle the homing sequence non-blocking
  
  switch (subState) {
    case 0:  // Check if motors are already at home
      // Logic for checking home position...
      subState = 1;
      break;
      
    case 1:  // Move motors away from home if needed
      // Logic for moving away from home...
      subState = 2;
      break;
      
    case 2:  // Home cut motor
      // Logic for homing cut motor...
      if (readCutMotorPositionSwitch()) {
        cutMotor.setCurrentPosition(0);
        subState = 3;
      }
      break;
      
    case 3:  // Home position motor
      // Logic for homing position motor...
      if (readPositionMotorPositionSwitch()) {
        positionMotor.setCurrentPosition(0);
        isHomingComplete = true;
        enterState(READY_STATE);
      }
      break;
  }
}
```

### Ready State

```cpp
void handleReadyState() {
  // Set the green LED on to indicate ready status
  digitalWrite(GREEN_LED_PIN, HIGH);
  
  // Check if reload switch is activated
  if (readReloadSwitch()) {
    enterState(RELOAD_STATE);
    return;
  }
  
  // Check if cycle switch is activated and ready to cycle
  if (readCycleSwitch()) {
    // Check if needCycleSwitchToggle flag is set
    if (!needCycleSwitchToggle) {
      // System is ready to start the cutting cycle
      enterState(CUTTING_STATE);
      return;
    }
  }
  
  // If cycle switch was ON at startup, wait until it's toggled OFF then ON
  if (needCycleSwitchToggle && !readCycleSwitch()) {
    needCycleSwitchToggle = false;
  }
}
```

## Error Handling

The system has three different error states to handle various error conditions:

1. **ERROR_STATE**: General error handling
2. **WOOD_SUCTION_ERROR_STATE**: Handles errors related to wood suction
3. **CUT_MOTOR_HOME_ERROR_STATE**: Handles errors related to cut motor homing

Each error state has its own recovery procedure. The system may attempt automatic recovery in some cases, or require operator intervention (cycle switch toggle) in others.

## LED Status Indicators

Each state has a distinct LED pattern to indicate the current system status:

- STARTUP_STATE: Blue LED on
- HOMING_STATE: Blue LED blinking
- READY_STATE: Green LED on
- RELOAD_STATE: Blue LED on
- CUTTING_STATE: Yellow LED on
- YESWOOD_STATE: Green and Yellow LEDs on
- NOWOOD_STATE: Blue and Green LEDs on
- ERROR_STATE: Red LED blinking
- WOOD_SUCTION_ERROR_STATE: Specific Red LED pattern for wood suction error
- CUT_MOTOR_HOME_ERROR_STATE: Specific Red LED pattern for homing error

## Non-Blocking Operation

All state operations are implemented in a non-blocking manner. No delays are used during motor movements or other operations. The system uses state machines and timers to track progress without blocking the main loop. 