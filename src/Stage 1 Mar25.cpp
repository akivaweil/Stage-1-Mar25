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
  CUT_MOTOR_HOME_ERROR_STATE,
  POSITION_MOTOR_HOME_ERROR_STATE
};

enum ErrorType {
  NO_ERROR,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR,
  POSITION_MOTOR_HOME_ERROR
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
const float CUT_MOTOR_CUTTING_SPEED = 50.0;  // steps/sec - slower speed for precise cutting
const float POSITION_MOTOR_NORMAL_SPEED = 30000.0;  // steps/sec
const float POSITION_MOTOR_RETURN_SPEED = 30000.0;  // steps/sec
const float CUT_MOTOR_ACCELERATION = 2200.0;  // steps/sec²
const float POSITION_MOTOR_ACCELERATION = 30000.0;  // steps/sec²
const float CUT_MOTOR_HOMING_SPEED = 300.0;  // steps/sec
const float POSITION_MOTOR_HOMING_SPEED = 2000.0;  // steps/sec

// Constants for positions
const float WAS_WOOD_SUCTIONED_POSITION = 0.3;  // inches
const float TRANSFER_ARM_SIGNAL_POSITION = 7.2;  // inches
const unsigned long MOTOR_MOVE_TIMEOUT = 10000; // 10 seconds

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
void handlePositionMotorHomeErrorState();
void updateRedLEDErrorPattern(ErrorType errorType);
void updateLEDsForState(State currentState);
void moveCutMotorToPosition(float positionInches);
void movePositionMotorToPosition(float positionInches);
bool isMotorInPosition(AccelStepper& motor, float targetPosition);
void moveAwayThenHomeCutMotor();
void moveAwayThenHomePositionMotor();
void runHomingSequence();
void Motors_HOMING_settings();
void CutMotor_HOMING_settings();
void PositionMotor_HOMING_settings();
void Motors_NORMAL_settings();
void CutMotor_NORMAL_settings();
void PositionMotor_NORMAL_settings();
void Motors_RETURN_settings();
void CutMotor_RETURN_settings();
void PositionMotor_RETURN_settings();
void CutMotor_CUTTING_settings();
void ensureMotorsAtHome();
bool cycleToggleDetected();
void resetErrorAndHomeSystem();
void signalTransferArm(bool state);
void updateTransferArmSignal();
void setRedLed(bool state);
void setGreenLed(bool state);
void setYellowLed(bool state);
void setBlueLed(bool state);
void allLedsOff();
bool readCutMotorHomingSwitch();
bool readPositionMotorHomingSwitch();
bool readReloadSwitch();
bool readCycleSwitch();
bool isWoodPresent();
bool isWoodSuctionProper();
void extendPositionClamp();
void retractPositionClamp();
void extendWoodSecureClamp();
void retractWoodSecureClamp();

// State pattern implementation
typedef void (*StateHandler)();

// Array of function pointers - indexes match State enum values
StateHandler stateHandlers[] = {
  handleStartupState,
  handleHomingState,
  handleReadyState,
  handleReloadState,
  handleCuttingState,
  handleYesWoodState,
  handleNoWoodState,
  handleErrorState,
  handleWoodSuctionErrorState,
  handleCutMotorHomeErrorState,
  handlePositionMotorHomeErrorState
};

// Main state machine update function
void updateStateMachine() {
  // Direct lookup - no switch needed
  stateHandlers[currentState]();
}

// State transition function
void enterState(State newState) {
  // Exit actions for current state
  switch (currentState) {
    case CUTTING_STATE:
      signalTransferArm(LOW);
      break;
      
    case ERROR_STATE:
    case WOOD_SUCTION_ERROR_STATE:
    case CUT_MOTOR_HOME_ERROR_STATE:
    case POSITION_MOTOR_HOME_ERROR_STATE:
      setRedLed(false);
      break;
  }
  
  // Update state
  currentState = newState;
  stateStartTime = millis();
  subState = 0;  // Always reset subState when entering a new state
  subStateTimer = 0; // Reset substate timer as well
  
  // Entry actions for new state
  switch (newState) {
    case HOMING_STATE:
      homingAttemptCount = 0;
      isHomingComplete = false;
      isCutMotorHomed = false;
      isPositionMotorHomed = false;
      break;
      
    case READY_STATE:
      setGreenLed(true);
      break;
      
    case RELOAD_STATE:
      setBlueLed(true);
      retractPositionClamp();
      retractWoodSecureClamp();
      break;
      
    case CUTTING_STATE:
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      break;
      
    case ERROR_STATE:
      currentError = NO_ERROR;
      break;
      
    case WOOD_SUCTION_ERROR_STATE:
      currentError = WOOD_SUCTION_ERROR;
      break;
      
    case CUT_MOTOR_HOME_ERROR_STATE:
      currentError = CUT_MOTOR_HOME_ERROR;
      break;
      
    case POSITION_MOTOR_HOME_ERROR_STATE:
      currentError = POSITION_MOTOR_HOME_ERROR;
      break;
  }
}

// Handle startup state - immediately transition to homing
void handleStartupState() {
  extendPositionClamp();
  extendWoodSecureClamp();
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
  enterState(HOMING_STATE);
}

// Handle homing state - implements a non-blocking homing sequence
void handleHomingState() {
  switch (subState) {
    case 0:  // Check if motors are already at home
      if (readCutMotorHomingSwitch()) {
        subState = 1;  // Need to move cut motor away from home
      } else {
        subState = 2;  // Cut motor already away from home, check position motor
      }
      
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
      CutMotor_HOMING_settings();
      
      if (!cutMotor.isRunning()) {
        cutMotor.move(-300);  // Move 300 steps away
      }
      
      if (!readCutMotorHomingSwitch() || cutMotor.distanceToGo() == 0) {
        cutMotor.stop();
        
        if (readPositionMotorHomingSwitch()) {
          subState = 3;
        } else {
          subState = 4;
        }
      }
      break;
      
    case 2:  // Intermediate case - should never reach here
      subState = (readPositionMotorHomingSwitch()) ? 3 : 4;
      break;
      
    case 3:  // Move position motor away from home switch
      PositionMotor_HOMING_settings();
      
      if (!positionMotor.isRunning()) {
        positionMotor.move(-300);  // Move 300 steps away
      }
      
      if (!readPositionMotorHomingSwitch() || positionMotor.distanceToGo() == 0) {
        positionMotor.stop();
        subState = 4;
      }
      break;
      
    case 4:  // Home cut motor
      CutMotor_HOMING_settings();
      
      if (!cutMotor.isRunning() && !readCutMotorHomingSwitch()) {
        cutMotor.move(15000);  // Move enough steps to reach home
      }
      
      if (readCutMotorHomingSwitch()) {
        cutMotor.stop();
        cutMotor.setCurrentPosition(0);
        isCutMotorHomed = true;
        subState = 5;
      } else if (cutMotor.distanceToGo() == 0) {
        homingAttemptCount++;
        
        if (homingAttemptCount >= 3) {
          enterState(CUT_MOTOR_HOME_ERROR_STATE);
          return;
        } else {
          subState = 1;
        }
      }
      break;
      
    case 5:  // Home position motor
      PositionMotor_HOMING_settings();
      
      if (!positionMotor.isRunning() && !readPositionMotorHomingSwitch()) {
        positionMotor.move(10000);  // Move enough steps to reach home
      }
      
      if (readPositionMotorHomingSwitch()) {
        positionMotor.stop();
        positionMotor.setCurrentPosition(0);
        isPositionMotorHomed = true;
        isHomingComplete = true;
        enterState(READY_STATE);
      } else if (positionMotor.distanceToGo() == 0) {
        homingAttemptCount++;
        
        if (homingAttemptCount >= 3) {
          enterState(POSITION_MOTOR_HOME_ERROR_STATE);
          return;
        } else {
          subState = 3;
        }
      }
      break;
  }
}

// Handle ready state - checks cycle switch toggle requirement
void handleReadyState() {
  setGreenLed(true);
  
  if (readReloadSwitch()) {
    enterState(RELOAD_STATE);
    return;
  }
  
  if (readCycleSwitch()) {
    if (needCycleSwitchToggle) {
      needCycleSwitchToggle = false;
      return;
    }
    
    enterState(CUTTING_STATE);
    return;
  }
  
  if (needCycleSwitchToggle && !readCycleSwitch()) {
    needCycleSwitchToggle = false;
  }
}

// Handle reload state - allows user to load new wood
void handleReloadState() {
  retractPositionClamp();
  retractWoodSecureClamp();
  setBlueLed(true);
  
  if (!readReloadSwitch()) {
    enterState(READY_STATE);
  }
}

// Handle cutting state - implements a non-blocking cutting sequence
void handleCuttingState() {
  static bool hasPastSuctionPosition = false;
  static bool hasPastTransferArmPosition = false;
  
  switch (subState) {
    case 0:  // Init cutting cycle
      extendPositionClamp();
      extendWoodSecureClamp();
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      hasPastSuctionPosition = false;
      hasPastTransferArmPosition = false;
      
      // Set up for continuous motion to final position
      CutMotor_CUTTING_settings();
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(CUT_MOTOR_TRAVEL_DISTANCE);
      }
      
      subState = 1;
      break;
      
    case 1:  // Monitor position during continuous movement and trigger actions at specific points
      // Check if we've passed the suction check position
      if (!hasSuctionBeenChecked && !hasPastSuctionPosition && 
          cutMotor.currentPosition() >= WAS_WOOD_SUCTIONED_POSITION * CUT_MOTOR_STEPS_PER_INCH) {
        
        hasPastSuctionPosition = true;
        
        // Check suction without stopping the motor
        if (isWoodSuctionProper()) {
          hasSuctionBeenChecked = true;
        } else {
          // Stop the motor and transition to error state if suction fails
          cutMotor.stop();
          enterState(WOOD_SUCTION_ERROR_STATE);
          return;
        }
      }
      
      // Check if we've passed the transfer arm signal position
      if (hasSuctionBeenChecked && !hasTransferArmBeenSignaled && !hasPastTransferArmPosition && 
          cutMotor.currentPosition() >= TRANSFER_ARM_SIGNAL_POSITION * CUT_MOTOR_STEPS_PER_INCH) {
        
        hasPastTransferArmPosition = true;
        
        // Signal transfer arm without stopping the motor
        signalTransferArm(HIGH);
        hasTransferArmBeenSignaled = true;
      }
      
      // Check if we've completed the full travel
      if (isMotorInPosition(cutMotor, CUT_MOTOR_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH)) {
        subState = 2;
      }
      break;
      
    case 2:  // Check wood presence after completing the cut
      if (isWoodPresent()) {
        enterState(YESWOOD_STATE);
      } else {
        enterState(NOWOOD_STATE);
      }
      break;
  }
}

// Handle yes wood state - wood was detected after cutting
void handleYesWoodState() {
  switch (subState) {
    case 0:  // Prepare for next position
      retractPositionClamp();
      PositionMotor_NORMAL_settings();
      subState = 1;
      break;
      
    case 1:  // Move position motor for next cut
      if (!positionMotor.isRunning()) {
        movePositionMotorToPosition(POSITION_MOTOR_TRAVEL_DISTANCE);
      }
      
      if (isMotorInPosition(positionMotor, POSITION_MOTOR_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH)) {
        subState = 2;
      }
      break;
      
    case 2:  // Return cut motor to home
      CutMotor_RETURN_settings();
      
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(0);
      }
      
      if (isMotorInPosition(cutMotor, 0)) {
        enterState(READY_STATE);
      }
      break;
  }
}

// Handle no wood state - no wood was detected after cutting
void handleNoWoodState() {
  switch (subState) {
    case 0:  // Prepare for return to home
      retractPositionClamp();
      retractWoodSecureClamp();
      Motors_RETURN_settings();
      subState = 1;
      break;
      
    case 1:  // Return cut motor to home
      if (!cutMotor.isRunning()) {
        moveCutMotorToPosition(0);
      }
      
      if (isMotorInPosition(cutMotor, 0)) {
        subState = 2;
      }
      break;
      
    case 2:  // Return position motor to home
      if (!positionMotor.isRunning()) {
        movePositionMotorToPosition(0);
      }
      
      if (isMotorInPosition(positionMotor, 0)) {
        enterState(READY_STATE);
      }
      break;
  }
}

// Handle error state
void handleErrorState() {
  updateRedLEDErrorPattern(currentError);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  ensureMotorsAtHome();
  
  if (cycleToggleDetected()) {
    resetErrorAndHomeSystem();
  }
}

// Handle wood suction error state
void handleWoodSuctionErrorState() {
  updateRedLEDErrorPattern(WOOD_SUCTION_ERROR);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  ensureMotorsAtHome();
  
  if (cycleToggleDetected()) {
    resetErrorAndHomeSystem();
  }
}

// Handle cut motor home error state
void handleCutMotorHomeErrorState() {
  updateRedLEDErrorPattern(CUT_MOTOR_HOME_ERROR);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  switch (subState) {
    case 0:  // First recovery attempt
      moveAwayThenHomeCutMotor();
      
      if (readCutMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 1;
      break;
      
    case 1:  // Second recovery attempt
      moveAwayThenHomeCutMotor();
      
      if (readCutMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 2;
      break;
      
    case 2:  // Locked state - requires cycle switch toggle
      bool currentCycleSwitchState = readCycleSwitch();
      
      if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
      }
      
      prevCycleSwitchState = currentCycleSwitchState;
      break;
  }
}

// Handle position motor home error state
void handlePositionMotorHomeErrorState() {
  updateRedLEDErrorPattern(POSITION_MOTOR_HOME_ERROR);
  
  extendPositionClamp();
  extendWoodSecureClamp();
  
  switch (subState) {
    case 0:  // First recovery attempt
      moveAwayThenHomePositionMotor();
      
      if (readPositionMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 1;
      break;
      
    case 1:  // Second recovery attempt
      moveAwayThenHomePositionMotor();
      
      if (readPositionMotorHomingSwitch()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
        return;
      }
      
      homingAttemptCount++;
      subState = 2;
      break;
      
    case 2:  // Locked state - requires cycle switch toggle
      if (cycleToggleDetected()) {
        homingAttemptCount = 0;
        currentError = NO_ERROR;
        enterState(HOMING_STATE);
      }
      break;
  }
}

// Helper function for cut motor homing recovery attempts
void moveAwayThenHomeCutMotor() {
  static unsigned long startTime = 0;
  static int recoverySubState = 0;
  
  switch (recoverySubState) {
    case 0:  // Start moving away from home
      CutMotor_HOMING_settings();
      
      if (!cutMotor.isRunning()) {
        cutMotor.move(-500);  // Move 500 steps away
        startTime = millis();
        recoverySubState = 1;
      }
      break;
      
    case 1:  // Wait for move away to complete or timeout
      if (!cutMotor.isRunning() || (millis() - startTime > 2000)) {
        cutMotor.stop();
        recoverySubState = 2;
      }
      break;
      
    case 2:  // Start moving toward home
      CutMotor_HOMING_settings();
      
      if (!cutMotor.isRunning()) {
        cutMotor.move(2000);  // Move 2000 steps toward home
        recoverySubState = 3;
      }
      break;
      
    case 3:  // Wait for home position or movement to complete
      if (readCutMotorHomingSwitch()) {
        cutMotor.stop();
        cutMotor.setCurrentPosition(0);
        recoverySubState = 0;
      } else if (!cutMotor.isRunning()) {
        recoverySubState = 0;
      }
      break;
  }
}

// Helper function for position motor homing recovery attempts
void moveAwayThenHomePositionMotor() {
  static unsigned long startTime = 0;
  static int recoverySubState = 0;
  
  switch (recoverySubState) {
    case 0:  // Start moving away from home
      PositionMotor_HOMING_settings();
      
      if (!positionMotor.isRunning()) {
        positionMotor.move(-500);  // Move 500 steps away
        startTime = millis();
        recoverySubState = 1;
      }
      break;
      
    case 1:  // Wait for move away to complete or timeout
      if (!positionMotor.isRunning() || (millis() - startTime > 2000)) {
        positionMotor.stop();
        recoverySubState = 2;
      }
      break;
      
    case 2:  // Start moving toward home
      PositionMotor_HOMING_settings();
      
      if (!positionMotor.isRunning()) {
        positionMotor.move(2000);  // Move 2000 steps toward home
        recoverySubState = 3;
      }
      break;
      
    case 3:  // Wait for home position or movement to complete
      if (readPositionMotorHomingSwitch()) {
        positionMotor.stop();
        positionMotor.setCurrentPosition(0);
        recoverySubState = 0;
      } else if (!positionMotor.isRunning()) {
        recoverySubState = 0;
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

void setPositionMotorHomeErrorPattern() {
  // Triple pulse pattern for position motor homing error: 200ms on, 200ms off, 200ms on, 200ms off, 200ms on, 400ms off
  unsigned long cycleTime = millis() % 1400;
  
  if (cycleTime < 200) {
    setRedLed(true);
  } else if (cycleTime < 400) {
    setRedLed(false);
  } else if (cycleTime < 600) {
    setRedLed(true);
  } else if (cycleTime < 800) {
    setRedLed(false);
  } else if (cycleTime < 1000) {
    setRedLed(true);
  } else {
    setRedLed(false);
  }
}

void setGeneralErrorPattern() {
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
      
    case POSITION_MOTOR_HOME_ERROR:
      setPositionMotorHomeErrorPattern();
      break;
      
    case NO_ERROR:
    default:
      setGeneralErrorPattern();
      break;
  }
}

// Update the LEDs based on the current state
void updateLEDsForState(State currentState) {
  allLedsOff();
  
  switch (currentState) {
    case STARTUP_STATE:
      setBlueLed(true);
      break;
      
    case HOMING_STATE:
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
    case POSITION_MOTOR_HOME_ERROR_STATE:
      // Error states LED patterns are handled by updateRedLEDErrorPattern()
      break;
  }
}

// Function to signal the transfer arm (non-blocking 500ms HIGH pulse)
void signalTransferArm(bool state) {
  if (state == HIGH && !transferArmSignalActive) {
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, HIGH);
    transferArmSignalStartTime = millis();
    transferArmSignalActive = true;
  } else if (state == LOW) {
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
    transferArmSignalActive = false;
  }
}

// Function to update transfer arm signal timing (call this in loop)
void updateTransferArmSignal() {
  if (transferArmSignalActive && (millis() - transferArmSignalStartTime >= 500)) {
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
  
  needCycleSwitchToggle = false;
  return true;
}

// Configure motors with their speeds and acceleration
void configureMotors() {
  Motors_NORMAL_settings();
  
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

// Move cut motor to specified position in inches
void moveCutMotorToPosition(float positionInches) {
  long steps = positionInches * CUT_MOTOR_STEPS_PER_INCH;
  cutMotor.moveTo(steps);
}

// Move position motor to specified position in inches
void movePositionMotorToPosition(float positionInches) {
  long steps = positionInches * POSITION_MOTOR_STEPS_PER_INCH;
  positionMotor.moveTo(steps);
}

// Check if motor has reached target position with small tolerance
bool isMotorInPosition(AccelStepper& motor, float targetPosition) {
  return (abs(motor.currentPosition() - targetPosition) < 5) && !motor.isRunning();
}

void loop() {
  updateAllSwitches();
  runMotors();
  updateTransferArmSignal();
  updateStateMachine();
  updateLEDsForState(currentState);
}

// Arduino setup function
void setup() {
  initializePins();
  initializeDebounce();
  configureMotors();
  performStartupSafetyCheck();
  currentState = STARTUP_STATE;
}

// Motor speed and acceleration configuration functions

// Configure both motors for homing operations
void Motors_HOMING_settings() {
  CutMotor_HOMING_settings();
  PositionMotor_HOMING_settings();
}

// Configure cut motor for homing
void CutMotor_HOMING_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_HOMING_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for homing
void PositionMotor_HOMING_settings() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_HOMING_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure both motors for normal operations
void Motors_NORMAL_settings() {
  CutMotor_NORMAL_settings();
  PositionMotor_NORMAL_settings();
}

// Configure cut motor for normal operations
void CutMotor_NORMAL_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for normal operations
void PositionMotor_NORMAL_settings() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure both motors for return operations
void Motors_RETURN_settings() {
  CutMotor_RETURN_settings();
  PositionMotor_RETURN_settings();
}

// Configure cut motor for return operations
void CutMotor_RETURN_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Configure position motor for return operations
void PositionMotor_RETURN_settings() {
  positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Configure cut motor for cutting operations (specific speed for cutting)
void CutMotor_CUTTING_settings() {
  cutMotor.setMaxSpeed(CUT_MOTOR_CUTTING_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
}

// Ensures both motors return to home position
void ensureMotorsAtHome() {
  // Return motors to home position if they're not already there
  if (!isMotorInPosition(cutMotor, 0)) {
    moveCutMotorToPosition(0);
  }
  
  if (!isMotorInPosition(positionMotor, 0)) {
    movePositionMotorToPosition(0);
  }
}

// Detects a cycle switch toggle (OFF to ON)
bool cycleToggleDetected() {
  bool currentCycleSwitchState = readCycleSwitch();
  bool result = (prevCycleSwitchState == false && currentCycleSwitchState == true);
  prevCycleSwitchState = currentCycleSwitchState;
  return result;
}

// Resets error state and returns to homing
void resetErrorAndHomeSystem() {
  currentError = NO_ERROR;
  enterState(HOMING_STATE);
}
