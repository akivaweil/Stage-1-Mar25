# State Machine Implementation

## State Machine Template

The following code template demonstrates how to implement the state machine for the wood cutting machine:

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
      isCycleInProgress = true;
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
      // Enter wood suction error - setup specific error pattern
      currentError = WOOD_SUCTION_ERROR;
      break;
      
    case CUT_MOTOR_HOME_ERROR_STATE:
      // Enter cut motor home error - setup specific error pattern
      currentError = CUT_MOTOR_HOME_ERROR;
      homingAttemptCount = 0;
      break;
  }
}
```

## Example State Handler Implementation

Here's an example implementation of the cutting state handler:

```cpp
void handleCuttingState() {
  switch (subState) {
    case 0:  // Init cutting cycle
      digitalWrite(YELLOW_LED_PIN, HIGH);  // Turn on yellow LED
      
      // Verify clamps are extended
      if (!areClampsPropertyExtended()) {
        extendPositionClamp();
        extendWoodSecureClamp();
        break;  // Wait for clamps to extend fully
      }
      
      // Start cut motor moving to cutting position
      moveCutMotorToPosition(CUT_MOTOR_TRAVEL_DISTANCE);
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      hasWoodSensorBeenChecked = false;
      subState = 1;
      break;
      
    case 1:  // Monitor cutting progress
      // Check if cut motor has reached suction check position
      if (!hasSuctionBeenChecked && 
          stepsToInches(cutMotor.currentPosition(), CUT_MOTOR_STEPS_PER_INCH) >= WOOD_SUCTION_CHECK_POSITION) {
        // Check suction sensor
        if (!isWoodSuctionProper()) {
          enterState(WOOD_SUCTION_ERROR_STATE);
          return;
        }
        hasSuctionBeenChecked = true;
      }
      
      // Check if cut motor has reached transfer arm signal position
      if (!hasTransferArmBeenSignaled && 
          stepsToInches(cutMotor.currentPosition(), CUT_MOTOR_STEPS_PER_INCH) >= TRANSFER_ARM_SIGNAL_POSITION) {
        signalTransferArm(HIGH);  // Signal transfer arm
        hasTransferArmBeenSignaled = true;
      }
      
      // Check if cut motor has reached its target position
      if (isMotorInPosition(cutMotor, inchesToSteps(CUT_MOTOR_TRAVEL_DISTANCE, CUT_MOTOR_STEPS_PER_INCH))) {
        signalTransferArm(LOW);  // Turn off transfer arm signal
        
        // Check wood sensor to determine next state
        if (isWoodPresent()) {
          enterState(YESWOOD_STATE);
        } else {
          enterState(NOWOOD_STATE);
        }
      }
      break;
  }
}
```

## Using Sub-States for Complex State Handling

Each state can have multiple sub-states to handle complex sequences:

1. Initialize `subState = 0` when entering a new state
2. Increment `subState` as steps of the sequence are completed
3. Use `switch (subState)` within each state handler to manage the sequence

This approach allows complex sequences to be broken into manageable steps while maintaining the overall state machine structure. 