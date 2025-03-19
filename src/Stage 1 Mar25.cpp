#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

// Enumerations
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

enum ErrorType {
  NO_ERROR,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR
};

// Pin definitions
const int CUT_MOTOR_PULSE_PIN = 48;
const int CUT_MOTOR_DIR_PIN = 47;
const int POSITION_MOTOR_PULSE_PIN = 21;
const int POSITION_MOTOR_DIR_PIN = 20;
const int CUT_MOTOR_HOMING_SWITCH_PIN = 10;
const int POSITION_MOTOR_HOMING_SWITCH_PIN = 9;
const int RELOAD_SWITCH_PIN = 14;
const int CYCLE_SWITCH_PIN = 13;
const int YES_OR_NO_WOOD_SENSOR_PIN = 11;
const int WAS_WOOD_SUCTIONED_SENSOR_PIN = 8;
const int POSITION_CLAMP_PIN = 18;
const int WOOD_SECURE_CLAMP_PIN = 17;
const int RED_LED_PIN = 7;
const int YELLOW_LED_PIN = 6;
const int GREEN_LED_PIN = 16;
const int BLUE_LED_PIN = 15;
const int SIGNAL_TO_TRANSFER_ARM_PIN = 19;

// Global variables
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;
unsigned long stateStartTime = 0;
unsigned long subStateTimer = 0;
int homingAttemptCount = 0;
int subState = 0;
bool isHomingComplete = false;
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;

// Motor instances
AccelStepper cutMotor(AccelStepper::DRIVER, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(AccelStepper::DRIVER, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

// Switch debouncing
Bounce cutMotorHomingSwitch = Bounce();
Bounce positionMotorHomingSwitch = Bounce();
Bounce reloadSwitch = Bounce();
Bounce cycleSwitch = Bounce();
Bounce yesOrNoWoodSensor = Bounce();
Bounce wasWoodSuctionedSensor = Bounce();

// Global variable for transfer arm signal timing
unsigned long transferArmSignalStartTime = 0;
bool transferArmSignalActive = false;

// Boolean to track cycle switch state for error recovery
bool prevCycleSwitchState = false;

// Global variable to track if cycle switch needs to be toggled before starting
bool needCycleSwitchToggle = false;

// Constants
const unsigned long DEBOUNCE_TIME = 15; // ms for switch debouncing

// Motor constants
const float CUT_MOTOR_STEPS_PER_INCH = 63.5;
const float POSITION_MOTOR_STEPS_PER_INCH = 1000.0;
const float CUT_MOTOR_TRAVEL_DISTANCE = 8.5;  // inches
const float POSITION_MOTOR_TRAVEL_DISTANCE = 3.45;  // inches
const float CUT_MOTOR_NORMAL_SPEED = 80.0;  // steps/sec
const float CUT_MOTOR_RETURN_SPEED = 2000.0;  // steps/sec
const float POSITION_MOTOR_NORMAL_SPEED = 30000.0;  // steps/sec
const float POSITION_MOTOR_RETURN_SPEED = 30000.0;  // steps/sec
const float CUT_MOTOR_ACCELERATION = 2200.0;  // steps/sec²
const float POSITION_MOTOR_ACCELERATION = 30000.0;  // steps/sec²
const float CUT_MOTOR_HOMING_SPEED = 300.0;  // steps/sec
const float POSITION_MOTOR_HOMING_SPEED = 2000.0;  // steps/sec

// Function declarations
void handleStartupState();
void handleHomingState();
void handleReadyState();
void handleReloadState();
void handleCuttingState();
void handleYesWoodState();
void handleNoWoodState();
void handleErrorState();
void handleWoodSuctionErrorState();
void handleCutMotorHomeErrorState();
void updateRedLEDErrorPattern(ErrorType errorType);
void updateLEDsForState(State currentState);
void moveCutMotorToPosition(float positionInches);
void movePositionMotorToPosition(float positionInches);
bool isMotorInPosition(AccelStepper& motor, float targetPosition);
void moveAwayThenHomeCutMotor();
void runHomingSequence();
void configureMotorsForHoming();
void configureCutMotorForHoming();
void configurePositionMotorForHoming();
void configureMotorsForNormalOperation();
void configureCutMotorForNormalOperation();
void configurePositionMotorForNormalOperation();
void configureMotorsForReturn();
void configureCutMotorForReturn();
void configurePositionMotorForReturn();

// Main state machine update function
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
      setRedLed(false);
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
      // Reset homing variables when entering homing state
      homingAttemptCount = 0;
      isHomingComplete = false;
      isCutMotorHomed = false;
      isPositionMotorHomed = false;
      break;
      
    case READY_STATE:
      // Turn on green LED
      setGreenLed(true);
      break;
      
    case RELOAD_STATE:
      // Turn on blue LED for reload
      setBlueLed(true);
      // Retract clamps to allow loading wood
      retractPositionClamp();
      retractWoodSecureClamp();
      break;
      
    case CUTTING_STATE:
      // Reset variables for cutting state
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      break;
      
    case ERROR_STATE:
      // Default error state
      currentError = NO_ERROR;
      break;
      
    case WOOD_SUCTION_ERROR_STATE:
      // Set the specific error type
      currentError = WOOD_SUCTION_ERROR;
      break;
      
    case CUT_MOTOR_HOME_ERROR_STATE:
      // Set the specific error type
      currentError = CUT_MOTOR_HOME_ERROR;
      break;
  }
}

// Handle startup state - immediately transition to homing
void handleStartupState() {
  // Initialize system by setting all clamps to their default positions
  extendPositionClamp();
  extendWoodSecureClamp();

  // Set initial state for Transfer Arm signal
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);

  // Immediately transition to homing state
  enterState(HOMING_STATE);
}

// Handle homing state - implements a non-blocking homing sequence
void handleHomingState() {
  // Implementation for the homing state as a state machine
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
      configureCutMotorForHoming();
      
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
      configurePositionMotorForHoming();
      
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
      configureCutMotorForHoming();
      
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
      configurePositionMotorForHoming();
      
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

// Handle ready state - checks cycle switch toggle requirement
void handleReadyState() {
  // Set the green LED on to indicate ready status
  setGreenLed(true);
  
  // Check if reload switch is activated
  if (readReloadSwitch()) {
    enterState(RELOAD_STATE);
    return;
  }
  
  // Check if cycle switch is activated and ready to cycle
  if (readCycleSwitch()) {
    // If needCycleSwitchToggle is true, we need to see the cycle switch go OFF first
    if (needCycleSwitchToggle) {
      // Wait until the cycle switch is toggled OFF then ON again
      needCycleSwitchToggle = false;  // Reset the flag since we've seen it go OFF
      return;  // Don't start cycle yet
    }
    
    // System is ready to start the cutting cycle
    enterState(CUTTING_STATE);
    return;
  }
  
  // If we needed a toggle and now cycle switch is OFF, clear the flag
  if (needCycleSwitchToggle && !readCycleSwitch()) {
    needCycleSwitchToggle = false;
  }
}

// Handle reload state - allows user to load new wood
void handleReloadState() {
  // Ensure clamps are retracted for loading
  retractPositionClamp();
  retractWoodSecureClamp();
  
  // Keep blue LED on for reload state
  setBlueLed(true);
  
  // Check if reload switch is deactivated
  if (!readReloadSwitch()) {
    // Return to ready state when reload switch is released
    enterState(READY_STATE);
  }
}

// Handle cutting state - implements a non-blocking cutting sequence
void handleCuttingState() {
  // Implementation for the cutting state
  // This is a state machine within a state machine
  
  switch (subState) {
    case 0:  // Init cutting cycle
      // Initialize by ensuring clamps are extended
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // Reset control flags
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      
      // Proceed to next substate
      subState = 1;
      break;
      
    case 1:  // Move to suction check position
      // Set motor speed
      configureCutMotorForNormalOperation();
      
      // Start move to suction check position if not already moving
      if (!cutMotor.isRunning()) {
        // Use 0.3 inches as the wood suction check position
        moveCutMotorToPosition(0.3);
      }
      
      // Check if position reached
      if (isMotorInPosition(cutMotor, 0.3 * CUT_MOTOR_STEPS_PER_INCH)) {
        subState = 2;  // Proceed to suction check
      }
      break;
      
    case 2:  // Check suction
      // Check if wood is properly suctioned (sensor reads HIGH when proper)
      if (isWoodSuctionProper()) {
        // Wood suction is proper, continue to next step
        hasSuctionBeenChecked = true;
        subState = 3;  // Continue cutting
      } else {
        // Wood suction failed, go to error state
        enterState(WOOD_SUCTION_ERROR_STATE);
        return;
      }
      break;
      
    case 3:  // Continue cutting to transfer arm signal position
      // Start move to transfer arm signal position
      // Use 7.2 inches as the transfer arm signal position
      moveCutMotorToPosition(7.2);
      
      // Check if position reached
      if (isMotorInPosition(cutMotor, 7.2 * CUT_MOTOR_STEPS_PER_INCH)) {
        subState = 4;  // Proceed to signal transfer arm
      }
      break;
      
    case 4:  // Signal transfer arm
      // Signal transfer arm if not already done
      if (!hasTransferArmBeenSignaled) {
        signalTransferArm(HIGH);
        hasTransferArmBeenSignaled = true;
      }
      
      // Allow 500ms for the signal (handled by signalTransferArm function)
      // Then continue cutting - non-blocking handled by updateTransferArmSignal in main loop
      subState = 5;
      break;
      
    case 5:  // Complete cut
      // Move to complete cut position
      moveCutMotorToPosition(CUT_MOTOR_TRAVEL_DISTANCE);
      
      // Check if cut completed
      if (isMotorInPosition(cutMotor, CUT_MOTOR_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH)) {
        subState = 6;  // Proceed to check for wood presence
      }
      break;
      
    case 6:  // Check wood presence
      // Check if wood is present at the end of cut cycle
      if (isWoodPresent()) {
        // Wood detected - transition to yeswood state
        enterState(YESWOOD_STATE);
      } else {
        // No wood detected - transition to nowood state
        enterState(NOWOOD_STATE);
      }
      break;
  }
}

// Handle yes wood state - wood was detected after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0:  // Prepare for next position
      // Keep wood secure clamp extended, retract position clamp
      retractPositionClamp();
      
      // Set position motor speed
      configurePositionMotorForNormalOperation();
      
      subState = 1;
      break;
      
    case 1:  // Move position motor for next cut
      // Start movement if not already moving
      if (!positionMotor.isRunning()) {
        movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE);
      }
      
      // Check if position reached
      if (isMotorInPosition(positionMotor, POSITION_MOTOR_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH)) {
        subState = 2;
      }
      break;
      
    case 2:  // Return cut motor to home
      // Set cut motor speed for return
      configureCutMotorForReturn();
      
      // Start movement if not already moving
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(0);
      }
      
      // Check if position reached
      if (isMotorInPosition(cutMotor, 0)) {
        // Return to ready state
        enterState(READY_STATE);
      }
      break;
  }
}

// Handle no wood state - no wood was detected after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0:  // Prepare for return to home
      // Retract both clamps
      retractPositionClamp();
      retractWoodSecureClamp();
      
      // Set motor speeds for return
      configureMotorsForReturn();
      
      subState = 1;
      break;
      
    case 1:  // Return cut motor to home
      // Start movement if not already moving
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(0);
      }
      
      // Check if position reached
      if (isMotorInPosition(cutMotor, 0)) {
        subState = 2;
      }
      break;
      
    case 2:  // Return position motor to home
      // Start movement if not already moving
      if (!positionMotor.isRunning()) {
        movePositionMotorToPosition(0);
      }
      
      // Check if position reached
      if (isMotorInPosition(positionMotor, 0)) {
        // Return to ready state
        enterState(READY_STATE);
      }
      break;
  }
}

// Handle error state
void handleErrorState() {
  // Update error LED - use a default error pattern
  updateRedLEDErrorPattern(currentError);
  
  // Keep clamps extended for safety
  extendPositionClamp();
  extendWoodSecureClamp();
  
  // Return motors to home position if they're not already there
  if (!isMotorInPosition(cutMotor, 0)) {
    moveCutMotorToPosition(0);
  }
  
  if (!isMotorInPosition(positionMotor, 0)) {
    movePositionMotorToPosition(0);
  }
  
  // Monitor cycle switch for toggle (OFF then ON)
  bool currentCycleSwitchState = readCycleSwitch();
  
  // Check for a complete toggle cycle (OFF then ON)
  if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
    // Cycle switch has been toggled off and then on again
    // Reset error and go to homing state
    currentError = NO_ERROR;
    enterState(HOMING_STATE);
  }
  
  // Update the previous cycle switch state
  prevCycleSwitchState = currentCycleSwitchState;
}

// Handle wood suction error state
void handleWoodSuctionErrorState() {
  // Keep updating the error LED pattern
  updateRedLEDErrorPattern(WOOD_SUCTION_ERROR);
  
  // Keep clamps extended for safety
  extendPositionClamp();
  extendWoodSecureClamp();
  
  // Return motors to home position if they're not already there
  if (!isMotorInPosition(cutMotor, 0)) {
    moveCutMotorToPosition(0);
  }
  
  if (!isMotorInPosition(positionMotor, 0)) {
    movePositionMotorToPosition(0);
  }
  
  // Monitor cycle switch for toggle (OFF then ON)
  bool currentCycleSwitchState = readCycleSwitch();
  
  // Check for a complete toggle cycle (OFF then ON)
  if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
    // Cycle switch has been toggled off and then on again
    // Reset error and go to homing state
    currentError = NO_ERROR;
    enterState(HOMING_STATE);
  }
  
  // Update the previous cycle switch state
  prevCycleSwitchState = currentCycleSwitchState;
}

// Handle cut motor home error state
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

// Helper function for cut motor homing recovery attempts
void moveAwayThenHomeCutMotor() {
  static unsigned long startTime = 0;
  static int recoverySubState = 0;
  
  switch (recoverySubState) {
    case 0:  // Start moving away from home
      // Set motor speed for moving away
      configureCutMotorForHoming();
      
      // Move motor away from home switch (negative direction)
      if (!cutMotor.isRunning()) {
        cutMotor.move(-500);  // Move 500 steps away
        startTime = millis();
        recoverySubState = 1;
      }
      break;
      
    case 1:  // Wait for move away to complete or timeout
      if (!cutMotor.isRunning() || (millis() - startTime > 2000)) {
        // Stop motor explicitly
        cutMotor.stop();
        recoverySubState = 2;
      }
      break;
      
    case 2:  // Start moving toward home
      // Set motor speed for homing
      configureCutMotorForHoming();
      
      // Move toward home position (positive direction)
      if (!cutMotor.isRunning()) {
        cutMotor.move(2000);  // Move 2000 steps toward home
        recoverySubState = 3;
      }
      break;
      
    case 3:  // Wait for home position or movement to complete
      if (readCutMotorHomingSwitch()) {
        // Stop immediately when home position reached
        cutMotor.stop();
        
        // Set current position as zero
        cutMotor.setCurrentPosition(0);
        
        // Recovery complete
        recoverySubState = 0;  // Reset for next time
      } else if (!cutMotor.isRunning()) {
        // Failed to find home within distance
        recoverySubState = 0;  // Reset for next attempt
      }
      break;
  }
}

// LED Pattern Functions
void allLedsOff() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void setGreenLed(bool state) {
  digitalWrite(GREEN_LED_PIN, state ? HIGH : LOW);
}

void setBlueLed(bool state) {
  digitalWrite(BLUE_LED_PIN, state ? HIGH : LOW);
}

void setYellowLed(bool state) {
  digitalWrite(YELLOW_LED_PIN, state ? HIGH : LOW);
}

void setRedLed(bool state) {
  digitalWrite(RED_LED_PIN, state ? HIGH : LOW);
}

void setYesWoodPattern() {
  setGreenLed(true);
  setYellowLed(true);
}

void setNoWoodPattern() {
  setBlueLed(true);
  setGreenLed(true);
}

void updateBlueBlinkPattern() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;
  
  if (millis() - lastBlinkTime >= 500) {
    lastBlinkTime = millis();
    ledState = !ledState;
    setBlueLed(ledState);
  }
}

void setWoodSuctionErrorPattern() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;
  
  // Fast blinking for wood suction error: 200ms on, 200ms off
  if (millis() - lastBlinkTime >= 200) {
    lastBlinkTime = millis();
    ledState = !ledState;
    setRedLed(ledState);
  }
}

void setCutMotorHomeErrorPattern() {
  // Double pulse pattern for homing error: 200ms on, 200ms off, 200ms on, 600ms off
  unsigned long cycleTime = millis() % 1200;
  
  if (cycleTime < 200) {
    setRedLed(true);
  } else if (cycleTime < 400) {
    setRedLed(false);
  } else if (cycleTime < 600) {
    setRedLed(true);
  } else {
    setRedLed(false);
  }
}

void setGeneralErrorPattern() {
  // Solid red for general error
  setRedLed(true);
}

// Update the red LED error pattern based on error type
void updateRedLEDErrorPattern(ErrorType errorType) {
  switch (errorType) {
    case WOOD_SUCTION_ERROR:
      setWoodSuctionErrorPattern();
      break;
      
    case CUT_MOTOR_HOME_ERROR:
      setCutMotorHomeErrorPattern();
      break;
      
    case NO_ERROR:
    default:
      setGeneralErrorPattern();
      break;
  }
}

// Update the LEDs based on the current state
void updateLEDsForState(State currentState) {
  // Turn all LEDs off first
  allLedsOff();
  
  // Set appropriate LED based on state
  switch (currentState) {
    case STARTUP_STATE:
      setBlueLed(true);
      break;
      
    case HOMING_STATE:
      // Blue LED blinks during homing
      updateBlueBlinkPattern();
      break;
      
    case READY_STATE:
      setGreenLed(true);
      break;
      
    case RELOAD_STATE:
      setBlueLed(true);
      break;
      
    case CUTTING_STATE:
      setYellowLed(true);
      break;
      
    case YESWOOD_STATE:
      setYesWoodPattern();
      break;
      
    case NOWOOD_STATE:
      setNoWoodPattern();
      break;
      
    case ERROR_STATE:
    case WOOD_SUCTION_ERROR_STATE:
    case CUT_MOTOR_HOME_ERROR_STATE:
      // Error states LED patterns are handled by updateRedLEDErrorPattern()
      break;
  }
}

// Function to signal the transfer arm (non-blocking 500ms HIGH pulse)
void signalTransferArm(bool state) {
  if (state == HIGH && !transferArmSignalActive) {
    // Turn on the signal and mark start time
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, HIGH);
    transferArmSignalStartTime = millis();
    transferArmSignalActive = true;
  } else if (state == LOW) {
    // Immediately turn off the signal
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
    transferArmSignalActive = false;
  }
}

// Function to update transfer arm signal timing (call this in loop)
void updateTransferArmSignal() {
  if (transferArmSignalActive && (millis() - transferArmSignalStartTime >= 500)) {
    // 500ms has elapsed, turn off the signal
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
    transferArmSignalActive = false;
  }
}

// Function to initialize all pins
void initializePins() {
  // Configure motor control pins as outputs
  pinMode(CUT_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(CUT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_DIR_PIN, OUTPUT);
  
  // Configure switch and sensor pins with appropriate pull-up/down resistors
  pinMode(CUT_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLUP);    // Active HIGH
  pinMode(POSITION_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLUP); // Active HIGH
  pinMode(RELOAD_SWITCH_PIN, INPUT_PULLUP);                // Active HIGH
  pinMode(CYCLE_SWITCH_PIN, INPUT_PULLUP);                 // Active HIGH
  pinMode(YES_OR_NO_WOOD_SENSOR_PIN, INPUT_PULLUP);          // Active LOW
  pinMode(WAS_WOOD_SUCTIONED_SENSOR_PIN, INPUT_PULLUP);      // Active LOW (but reads HIGH when proper)
  
  // Configure output pins
  pinMode(POSITION_CLAMP_PIN, OUTPUT);
  pinMode(WOOD_SECURE_CLAMP_PIN, OUTPUT);
  pinMode(SIGNAL_TO_TRANSFER_ARM_PIN, OUTPUT);
  
  // Configure LED pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  // Initialize outputs to default states
  digitalWrite(POSITION_CLAMP_PIN, HIGH);           // Retracted
  digitalWrite(WOOD_SECURE_CLAMP_PIN, HIGH);        // Retracted
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);    // Inactive
  
  // Initialize all LEDs to off
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}

// Initialize switch debouncing with 15ms debounce time
void initializeDebounce() {
  // Attach and configure each switch with 15ms debounce time
  cutMotorHomingSwitch.attach(CUT_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLUP);
  cutMotorHomingSwitch.interval(DEBOUNCE_TIME);
  
  positionMotorHomingSwitch.attach(POSITION_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLUP);
  positionMotorHomingSwitch.interval(DEBOUNCE_TIME);
  
  reloadSwitch.attach(RELOAD_SWITCH_PIN, INPUT_PULLUP);
  reloadSwitch.interval(DEBOUNCE_TIME);
  
  cycleSwitch.attach(CYCLE_SWITCH_PIN, INPUT_PULLUP);
  cycleSwitch.interval(DEBOUNCE_TIME);
  
  yesOrNoWoodSensor.attach(YES_OR_NO_WOOD_SENSOR_PIN, INPUT_PULLUP);
  yesOrNoWoodSensor.interval(DEBOUNCE_TIME);
  
  wasWoodSuctionedSensor.attach(WAS_WOOD_SUCTIONED_SENSOR_PIN, INPUT_PULLUP);
  wasWoodSuctionedSensor.interval(DEBOUNCE_TIME);
}

// Update all switch readings
void updateAllSwitches() {
  cutMotorHomingSwitch.update();
  positionMotorHomingSwitch.update();
  reloadSwitch.update();
  cycleSwitch.update();
  yesOrNoWoodSensor.update();
  wasWoodSuctionedSensor.update();
}

// Sensor reading functions 
bool readCutMotorHomingSwitch() {
  // Active HIGH
  return cutMotorHomingSwitch.read() == HIGH;
}

bool readPositionMotorHomingSwitch() {
  // Active HIGH
  return positionMotorHomingSwitch.read() == HIGH;
}

bool readReloadSwitch() {
  // Active HIGH
  return reloadSwitch.read() == HIGH;
}

bool readCycleSwitch() {
  // Active HIGH
  return cycleSwitch.read() == HIGH;
}

bool isWoodPresent() {
  // Active LOW - wood is present when sensor reads LOW
  return yesOrNoWoodSensor.read() == LOW;
}

bool isWoodSuctionProper() {
  // Active LOW - suction is proper when sensor reads HIGH (inverted logic)
  return wasWoodSuctionedSensor.read() == HIGH;
}

// Clamp control functions - all operations are immediate with no delays
void extendPositionClamp() {
  // LOW signal = Clamp Extended/Engaged
  digitalWrite(POSITION_CLAMP_PIN, LOW);
}

void retractPositionClamp() {
  // HIGH signal = Clamp Retracted
  digitalWrite(POSITION_CLAMP_PIN, HIGH);
}

void extendWoodSecureClamp() {
  // LOW signal = Clamp Extended/Engaged
  digitalWrite(WOOD_SECURE_CLAMP_PIN, LOW);
}

void retractWoodSecureClamp() {
  // HIGH signal = Clamp Retracted
  digitalWrite(WOOD_SECURE_CLAMP_PIN, HIGH);
}

// Perform startup safety check - checks if cycle switch is on during startup
bool performStartupSafetyCheck() {
  // If cycle switch is ON at startup, require it to be toggled OFF then ON before allowing operation
  if (readCycleSwitch()) {
    needCycleSwitchToggle = true;
    return false;
  }
  
  // Cycle switch is OFF at startup, no toggle needed
  needCycleSwitchToggle = false;
  return true;
}

// Configure motors with their speeds and acceleration
void configureMotors() {
  // Use our new configuration functions instead of direct settings
  configureMotorsForNormalOperation();
  
  // Set current position to 0
  cutMotor.setCurrentPosition(0);
  positionMotor.setCurrentPosition(0);
}

// Run motors - call this in each loop iteration for non-blocking operation
void runMotors() {
  cutMotor.run();
  positionMotor.run();
}

// Convert inches to motor steps
float inchesToSteps(float inches, float stepsPerInch) {
  return inches * stepsPerInch;
}

// Convert motor steps to inches
float stepsToInches(long steps, float stepsPerInch) {
  return (float)steps / stepsPerInch;
}

void loop() {
  // Update all switch/sensor readings
  updateAllSwitches();
  
  // Run motors (non-blocking)
  runMotors();
  
  // Update transfer arm signal timing
  updateTransferArmSignal();
  
  // Update state machine
  updateStateMachine();
  
  // Update LEDs based on current state
  updateLEDsForState(currentState);
}

// Arduino setup function
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

// Motor speed and acceleration configuration functions

// Configure both motors for homing operations
void configureMotorsForHoming() {
  configureCutMotorForHoming();
  configurePositionMotorForHoming();
}

// Configure cut motor for homing
void configureCutMotorForHoming() {
  cutMotor.setMaxSpeed(CUT_MOTOR_HOMING_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for homing
void configurePositionMotorForHoming() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_HOMING_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure both motors for normal operations
void configureMotorsForNormalOperation() {
  configureCutMotorForNormalOperation();
  configurePositionMotorForNormalOperation();
}

// Configure cut motor for normal operations
void configureCutMotorForNormalOperation() {
  cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for normal operations
void configurePositionMotorForNormalOperation() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure both motors for return operations
void configureMotorsForReturn() {
  configureCutMotorForReturn();
  configurePositionMotorForReturn();
}

// Configure cut motor for return operations
void configureCutMotorForReturn() {
  cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for return operations
void configurePositionMotorForReturn() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}
