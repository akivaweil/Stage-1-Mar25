// Stage 1 Wood Cutting Machine
// Controls two stepper motors and two pneumatic clamps to precisely cut wood boards

#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

// ===== PIN DEFINITIONS =====

// Motor Pin Definitions
const int CUT_MOTOR_PULSE_PIN = 39;       // Controls the pulse signal to the cut motor driver - moves the sliding table saw carriage
const int CUT_MOTOR_DIR_PIN = 38;         // Controls the direction of the cut motor (forward/backward movement)

const int POSITION_MOTOR_PULSE_PIN = 1;  // Controls the pulse signal to the position motor driver - feeds wood forward for next cut
const int POSITION_MOTOR_DIR_PIN = 2;    // Controls the direction of the position motor (forward/backward movement)

// Switch and Sensor Pin Definitions (Left side inputs)
const int CUT_MOTOR_HOMING_SWITCH = 10;      // Limit switch that detects when cut motor is at home position
const int POSITION_MOTOR_HOMING_SWITCH = 9;  // Limit switch that detects when position motor is at home position
const int RELOAD_SWITCH = 14;                // Manual switch to enter reload mode - disengages clamps for material loading
const int WOOD_SENSOR = 11;                  // Sensor that detects if wood is present (LOW when wood is detected)

// Switch and Sensor Pin Definitions (Right side)
const int CYCLE_SWITCH = 13;              // Switch that initiates the cutting cycle when activated
const int WAS_WOOD_SUCTIONED_SENSOR = 8;  // Sensor that checks if wood was properly suctioned during cutting (error detection)

// Clamp Pin Definitions (LOW = Extended/Engaged, HIGH = Retracted)
const int POSITION_CLAMP = 18;           // Controls the pneumatic clamp that holds the positioning mechanism
const int WOOD_SECURE_CLAMP = 17;        // Controls the pneumatic clamp that secures the wood piece

// LED Pin Definitions
const int RED_LED = 7;     // Error indicator LED - blinks during error conditions
const int YELLOW_LED = 6;  // Operation in progress indicator - on during cutting or when in reload mode
const int GREEN_LED = 16;  // Ready indicator - on when system is ready to begin a cutting cycle
const int BLUE_LED = 15;   // Setup/special mode indicator - on during startup or when no wood present

// Signal Output
const int SIGNAL_TO_STAGE_1TO2 = 19;    // Output signal to the next stage in the process (Stage 2) when cutting is complete

// ===== CONSTANTS =====
// System States
enum SystemState {
  STARTUP,
  HOMING,
  READY,
  RELOAD,
  CUTTING,
  YESWOOD,
  NOWOOD,
  ERROR
};

// Error Types
enum ErrorType {
  NONE,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR
};

// Motor Calibration
const float CUT_MOTOR_STEPS_PER_INCH = 76.0;
const float POSITION_MOTOR_STEPS_PER_INCH = 1000.0;
const float CUT_MOTOR_TRAVEL_DISTANCE = 7.7;
const float POSITION_MOTOR_TRAVEL_DISTANCE = 3.45;

// Motor Speeds
const float CUT_MOTOR_NORMAL_SPEED = 105;
const float CUT_MOTOR_RETURN_SPEED = 1500;
const float POSITION_MOTOR_NORMAL_SPEED = 30000;
const float POSITION_MOTOR_RETURN_SPEED = 30000;

// Motor Acceleration
const float CUT_MOTOR_ACCELERATION = 2500;
const float POSITION_MOTOR_ACCELERATION = 30000;
const float POSITION_MOTOR_RETURN_ACCELERATION = 30000;

// Homing Speeds
const float CUT_MOTOR_HOMING_SPEED = 300;
const float POSITION_MOTOR_HOMING_SPEED = 2000;

// Signal Debounce Interval
const int SIGNAL_DEBOUNCE_INTERVAL = 10; // 10ms debounce

// Position Constants
const long CUT_MOTOR_FULL_TRAVEL = CUT_MOTOR_STEPS_PER_INCH * CUT_MOTOR_TRAVEL_DISTANCE;
const long POSITION_MOTOR_FULL_TRAVEL = POSITION_MOTOR_STEPS_PER_INCH * POSITION_MOTOR_TRAVEL_DISTANCE;
const long CUT_MOTOR_WOOD_SUCTION_CHECK_POS = CUT_MOTOR_STEPS_PER_INCH * 0.5;  // 0.5 inch
const long CUT_MOTOR_TA_SIGNAL_POS = CUT_MOTOR_STEPS_PER_INCH * 7.2;           // 7.2 inches
const long POSITION_MOTOR_CLAMP_RETRACT_POS = POSITION_MOTOR_STEPS_PER_INCH * 0.1; // 0.1 inch

// ===== STEPPER MOTOR OBJECTS =====
// 1 = step interface type (Step/Dir pins)
AccelStepper cutMotor(1, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(1, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

// ===== SYSTEM VARIABLES =====
SystemState currentState = STARTUP;
ErrorType currentError = NONE;
bool isNoWoodCycleCompleted = false;
bool signalSentToTA = false;
bool woodSuctionChecked = false;

// Flags
bool isHomed = false;
bool isReloadMode = false;
bool isWoodPresent = false;
bool hasWoodSuctionError = false;
bool hasHomingError = false;  // Flag to track homing errors
bool isSignalSent = false;
bool isErrorAcknowledged = false;
bool isCuttingCycleInProgress = false;
bool isContinuousModeActive = false;
bool requireCycleSwitchReset = false;  // Requires cycle switch to be reset (toggled off and back on) if it was ON at startup
bool hasCheckedForWoodSuction = false;

// Timers
unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long blinkStartTime = 0;
unsigned long lastErrorBlinkTime = 0;
unsigned long errorBlinkStartTime = 0;
unsigned long errorStartTime = 0;
unsigned long positionMoveStartTime = 0;
unsigned long delayStartTime = 0;

// LED States
bool blinkState = false;
bool errorBlinkState = false;

// ===== DEBOUNCE OBJECTS =====
Bounce cutHomingSensor = Bounce();
Bounce positionHomingSensor = Bounce();
Bounce reloadSwitch = Bounce();
Bounce cycleSwitch = Bounce();
Bounce woodSensor = Bounce();
Bounce woodSuctionSensor = Bounce();

// ===== FUNCTION PROTOTYPES =====
// Core System Functions
void setup();
void loop();
void initializeSystem();
void readSwitchStates();

// Machine Functions
void homingSequence();
void Ready();
void CUTmovement();
void YESwood();  // Renamed from positioningOperation
void NOwood();   // Renamed from noWoodOperation
void handleCutMotorHomingError();
void suctionError();

// Utility Functions
bool Wait(unsigned long delayTime, unsigned long* startTimePtr);
void handleStartup();
void handleHoming();
void handleReady();
void handleReload();
void handleCutting();
void handleYesWood();
void handleNoWood();
void handleError();
void setLEDs(bool red, bool yellow, bool green, bool blue);
void blinkLED(int pin, int interval);
void blinkErrorPattern();
void extendClamp(int clampPin);
void retractClamp(int clampPin);
bool isClampExtended(int clampPin);
bool isClampRetracted(int clampPin);
bool isCutMotorAtHome();
bool isPositionMotorAtHome();
bool checkWoodPresent();
bool isWoodSuctionCorrect();
bool isCycleSwitchOn();
bool isReloadSwitchOn();

// -------------------- UTILITY FUNCTIONS --------------------
// LED Control Functions
void setLEDs(bool red, bool yellow, bool green, bool blue) {
  digitalWrite(RED_LED, red ? HIGH : LOW);
  digitalWrite(YELLOW_LED, yellow ? HIGH : LOW);
  digitalWrite(GREEN_LED, green ? HIGH : LOW);
  digitalWrite(BLUE_LED, blue ? HIGH : LOW);
}

void blinkLED(int pin, int interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - lastBlinkTime >= interval) {
    lastBlinkTime = currentMillis;
    blinkState = !blinkState;
    digitalWrite(pin, blinkState ? HIGH : LOW);
  }
}

void blinkErrorPattern() {
  static enum {
    START_PATTERN,
    LED_ON,
    LED_OFF,
    PATTERN_PAUSE
  } patternState = START_PATTERN;
  
  static int flashCount = 0;
  const int totalFlashes = 3;
  const int onTime = 200;   // LED on time in ms
  const int offTime = 200;  // LED off time in ms
  const int pauseTime = 1000; // Pause time in ms
  
  switch(patternState) {
    case START_PATTERN:
      flashCount = 0;
      digitalWrite(RED_LED, LOW);
      errorBlinkStartTime = millis();
      patternState = LED_ON;
      break;
      
    case LED_ON:
      digitalWrite(RED_LED, HIGH);
      if (millis() - errorBlinkStartTime >= onTime) {
        errorBlinkStartTime = millis();
        patternState = LED_OFF;
      }
      break;
      
    case LED_OFF:
      digitalWrite(RED_LED, LOW);
      if (millis() - errorBlinkStartTime >= offTime) {
        flashCount++;
        if (flashCount < totalFlashes) {
          errorBlinkStartTime = millis();
          patternState = LED_ON;
        } else {
          errorBlinkStartTime = millis();
          patternState = PATTERN_PAUSE;
        }
      }
      break;
      
    case PATTERN_PAUSE:
      if (millis() - errorBlinkStartTime >= pauseTime) {
        patternState = START_PATTERN;
      }
      break;
  }
}

// Clamp Functions
void extendClamp(int clampPin) {
  digitalWrite(clampPin, LOW);  // Extend clamp (LOW = extended)
}

void retractClamp(int clampPin) {
  digitalWrite(clampPin, HIGH); // Retract clamp (HIGH = retracted)
}

bool isClampExtended(int clampPin) {
  return digitalRead(clampPin) == LOW;
}

bool isClampRetracted(int clampPin) {
  return digitalRead(clampPin) == HIGH;
}

// Sensor/Switch Check Functions
bool isCutMotorAtHome() {
  return cutHomingSensor.read() == HIGH; // Switch is HIGH when triggered
}

bool isPositionMotorAtHome() {
  return positionHomingSensor.read() == HIGH; // Switch is HIGH when triggered
}

bool checkWoodPresent() {
  return woodSensor.read() == LOW; // LOW when wood is present
}

bool isWoodSuctionCorrect() {
  return woodSuctionSensor.read() == LOW; // LOW when suction is correct
}

bool isCycleSwitchOn() {
  return cycleSwitch.read() == HIGH; // HIGH when switch is ON
}

bool isReloadSwitchOn() {
  return reloadSwitch.read() == HIGH; // HIGH when switch is ON
}

// -------------------- NON-BLOCKING DELAY FUNCTION --------------------
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

void setup() {
  // Initialize the system
  initializeSystem();
  
  // Initialize Serial for debugging
  // Serial.begin(115200);
  // Serial.println("Stage 1 system starting...");
  
  // Start in STARTUP state
  currentState = STARTUP;
  
  // Check if cycle switch is already ON at startup
  cycleSwitch.update();
  if (cycleSwitch.read() == HIGH) {
    requireCycleSwitchReset = true;  // Require switch to be reset before operation
    // Serial.println("WARNING: Cycle switch is ON at startup");
  } else {
    requireCycleSwitchReset = false;  // Switch is already OFF, no reset needed
  }
  
  // Brief delay before starting homing
  delay(10);
}

void initializeSystem() {
  // Initialize serial communication
  // Serial.begin(115200);
  // Serial.println("System initializing...");
  
  // Initialize motor pins
  pinMode(CUT_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(CUT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_DIR_PIN, OUTPUT);
  
  // Initialize switch and sensor pins
  pinMode(CUT_MOTOR_HOMING_SWITCH, INPUT_PULLUP);
  pinMode(POSITION_MOTOR_HOMING_SWITCH, INPUT_PULLUP);
  pinMode(RELOAD_SWITCH, INPUT);                // External pull-down
  pinMode(CYCLE_SWITCH, INPUT);                 // External pull-down
  pinMode(WOOD_SENSOR, INPUT_PULLUP);           // Active LOW (LOW = wood present)
  pinMode(WAS_WOOD_SUCTIONED_SENSOR, INPUT);    // External pull-down
  
  // Initialize clamp pins
  pinMode(POSITION_CLAMP, OUTPUT);
  pinMode(WOOD_SECURE_CLAMP, OUTPUT);
  
  // Initialize LED pins
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  // Initialize signal output pin
  pinMode(SIGNAL_TO_STAGE_1TO2, OUTPUT);
  
  // Set initial state of outputs
  digitalWrite(POSITION_CLAMP, LOW);      // Extend position clamp
  digitalWrite(WOOD_SECURE_CLAMP, LOW);   // Extend wood secure clamp
  digitalWrite(RED_LED, LOW);             // Turn off error LED
  digitalWrite(YELLOW_LED, LOW);          // Turn off busy LED
  digitalWrite(GREEN_LED, LOW);           // Turn off ready LED
  digitalWrite(BLUE_LED, HIGH);           // Turn on setup LED during startup
  digitalWrite(SIGNAL_TO_STAGE_1TO2, LOW);    // No signal to TA
  
  // Configure motors
  cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
  positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
  
  // Initialize bounce objects for debouncing switches
  cutHomingSensor.attach(CUT_MOTOR_HOMING_SWITCH);
  cutHomingSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  positionHomingSensor.attach(POSITION_MOTOR_HOMING_SWITCH);
  positionHomingSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  reloadSwitch.attach(RELOAD_SWITCH);
  reloadSwitch.interval(SIGNAL_DEBOUNCE_INTERVAL);
  cycleSwitch.attach(CYCLE_SWITCH);
  cycleSwitch.interval(SIGNAL_DEBOUNCE_INTERVAL);
  woodSensor.attach(WOOD_SENSOR);
  woodSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  woodSuctionSensor.attach(WAS_WOOD_SUCTIONED_SENSOR);
  woodSuctionSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  
  // Test and report homing sensor states
  // Serial.println("Initial sensor states:");
  // Serial.print("Cut homing sensor: ");
  // Serial.println(digitalRead(CUT_MOTOR_HOMING_SWITCH) == HIGH ? "HIGH (active)" : "LOW (inactive)");
  // Serial.print("Position homing sensor: ");
  // Serial.println(digitalRead(POSITION_MOTOR_HOMING_SWITCH) == HIGH ? "HIGH (active)" : "LOW (inactive)");
  
  // Set system as not homed
  isHomed = false;
}

void readSwitchStates() {
  // Update all debounced switches
  cutHomingSensor.update();
  positionHomingSensor.update();
  reloadSwitch.update();
  cycleSwitch.update();
  woodSensor.update();
  woodSuctionSensor.update();
  
  // Read wood sensor (active LOW)
  isWoodPresent = (digitalRead(WOOD_SENSOR) == LOW);
}

void loop() {
  // Read all switch states
  readSwitchStates();
  
  // Handle cycle switch safety check
  if (requireCycleSwitchReset && cycleSwitch.fell()) {
    // Cycle switch was turned OFF after being ON during startup
    requireCycleSwitchReset = false;
  }
  
  // Reset isNoWoodCycleCompleted flag when cycle switch is released
  if (isNoWoodCycleCompleted && cycleSwitch.fell()) {
    isNoWoodCycleCompleted = false;
  }
  
  // Handle the reload switch state when in READY state
  if (currentState == READY) {
    // Check current state of reload switch (HIGH = ON with pull-down resistor)
    bool isReloadSwitchOn = reloadSwitch.read() == HIGH;
    
    if (isReloadSwitchOn && !isReloadMode) {
      // Enter reload mode
      isReloadMode = true;
      digitalWrite(POSITION_CLAMP, HIGH); // Retract position clamp
      digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Retract wood secure clamp
      digitalWrite(YELLOW_LED, HIGH);     // Turn on blue and yellow LED for reload mode
      digitalWrite(BLUE_LED, HIGH);
    } else if (!isReloadSwitchOn && isReloadMode) {
      // Exit reload mode
      isReloadMode = false;
      digitalWrite(POSITION_CLAMP, LOW);   // Extend position clamp
      digitalWrite(WOOD_SECURE_CLAMP, LOW); // Extend wood secure clamp
      digitalWrite(YELLOW_LED, LOW);       // Turn off blue and yellow LED
      digitalWrite(BLUE_LED, LOW);
    }
  }
  
  // Handle error acknowledgment separately
  if (reloadSwitch.rose() && currentState == ERROR) {
    isErrorAcknowledged = true;
  }
  
  // Check for continuous mode activation/deactivation
  bool isCycleSwitchOn = cycleSwitch.read() == HIGH;
  if (isCycleSwitchOn != isContinuousModeActive && !requireCycleSwitchReset) {
    isContinuousModeActive = isCycleSwitchOn;
  }
  
  // State machine
  switch (currentState) {
    case STARTUP:
      digitalWrite(BLUE_LED, HIGH);  // Blue LED on during startup/homing
      currentState = HOMING;
      break;
      
    case HOMING:
      homingSequence();
      break;
      
    case READY:
      Ready();
      break;
      
    case CUTTING:
      CUTmovement();
      break;
      
    case YESWOOD:
      YESwood();
      break;
      
    case NOWOOD:
      NOwood();
      break;
      
    case ERROR:
      if (hasWoodSuctionError) {
        suctionError();
      }
      if (hasHomingError) {
        handleCutMotorHomingError();
      }
      break;
  }
  
  // Run the motors (non-blocking)
  cutMotor.run();
  positionMotor.run();
}

// -------------------- MACHINE OPERATION FUNCTIONS --------------------

// Homing Sequence
void homingSequence() {
  static enum {
    HOMING_START,
    MOVE_FROM_SWITCHES_IF_NEEDED,
    HOME_CUT_MOTOR,
    HOME_POSITION_MOTOR,
    MOVE_POSITION_TO_OPERATING,
    HOMING_COMPLETE
  } homingState = HOMING_START;
  
  // Blink blue LED during homing
  blinkLED(BLUE_LED, 500);
  
  switch(homingState) {
    case HOMING_START:
      // Initialize homing sequence
      // Serial.println("Starting homing sequence");  // Commented out as specified
      cutMotor.setMaxSpeed(CUT_MOTOR_HOMING_SPEED);
      positionMotor.setMaxSpeed(POSITION_MOTOR_HOMING_SPEED);
      homingState = MOVE_FROM_SWITCHES_IF_NEEDED;
      break;
      
    case MOVE_FROM_SWITCHES_IF_NEEDED:
      // If switches are already triggered, move away first
      if (isCutMotorAtHome()) {
        cutMotor.setCurrentPosition(0);
        cutMotor.moveTo(CUT_MOTOR_STEPS_PER_INCH * 1.0); // Move 1 inch away
      }
      
      if (isPositionMotorAtHome()) {
        positionMotor.setCurrentPosition(0);
        positionMotor.moveTo(POSITION_MOTOR_STEPS_PER_INCH * 1.0); // Move 1 inch away
      }
      
      // If both motors are at switches or both have started moving
      if ((!isCutMotorAtHome() || cutMotor.distanceToGo() != 0) && 
          (!isPositionMotorAtHome() || positionMotor.distanceToGo() != 0)) {
        homingState = HOME_CUT_MOTOR;
      }
      break;
      
    case HOME_CUT_MOTOR:
      // If motor is still moving away from switch
      if (cutMotor.distanceToGo() != 0) {
        // Let it move
        break;
      }
      
      // Start homing cut motor
      if (!isCutMotorAtHome()) {
        cutMotor.setSpeed(-CUT_MOTOR_HOMING_SPEED); // Move toward home switch (negative direction)
        cutMotor.runSpeed(); // Direct speed control for homing
      } else {
        // When switch is triggered, stop and set position to 0
        cutMotor.setSpeed(0);
        cutMotor.setCurrentPosition(0);
        homingState = HOME_POSITION_MOTOR;
      }
      break;
      
    case HOME_POSITION_MOTOR:
      // If motor is still moving away from switch
      if (positionMotor.distanceToGo() != 0) {
        // Let it move
        break;
      }
      
      // Start homing position motor
      if (!isPositionMotorAtHome()) {
        positionMotor.setSpeed(-POSITION_MOTOR_HOMING_SPEED); // Move toward home switch (negative direction)
        positionMotor.runSpeed(); // Direct speed control for homing
      } else {
        // When switch is triggered, stop and set position to 0
        positionMotor.setSpeed(0);
        positionMotor.setCurrentPosition(0);
        homingState = MOVE_POSITION_TO_OPERATING;
      }
      break;
      
    case MOVE_POSITION_TO_OPERATING:
      // Reset speeds back to normal
      cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
      positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
      
      // Move position motor to operating position
      positionMotor.moveTo(POSITION_MOTOR_FULL_TRAVEL);
      if (positionMotor.distanceToGo() == 0) {
        homingState = HOMING_COMPLETE;
      }
      break;
      
    case HOMING_COMPLETE:
      // Homing complete, extend both clamps
      extendClamp(POSITION_CLAMP);
      extendClamp(WOOD_SECURE_CLAMP);
      
      // Transition to READY state
      // Serial.println("Homing complete");  // Commented out as specified
      homingState = HOMING_START; // Reset for next time
      currentState = READY;
      break;
  }
}

// Ready state handler
void Ready() {
  // Set LEDs - Green ON, others OFF
  setLEDs(false, false, true, false);
  
  // Check reload switch
  if (isReloadSwitchOn()) {
    currentState = RELOAD;
    return;
  }
  
  // Check cycle switch and if wood is present to start cycle
  if (isCycleSwitchOn() && checkWoodPresent()) {
    // Reset cycle variables
    signalSentToTA = false;
    woodSuctionChecked = false;
    currentState = CUTTING;
  }
}

// Cut movement state handler
void CUTmovement() {
  static enum {
    INIT_CUTTING,
    VERIFY_CLAMPS,
    EXECUTE_CUT,
    CHECK_WOOD
  } cuttingState = INIT_CUTTING;
  
  switch(cuttingState) {
    case INIT_CUTTING:
      // Initialize cutting cycle
      // Serial.println("Starting cutting cycle");  // Commented out as specified
      
      // Set yellow LED ON to indicate operation in progress
      setLEDs(false, true, false, false);
      
      // Prepare for cutting
      cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
      cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
      
      cuttingState = VERIFY_CLAMPS;
      break;
      
    case VERIFY_CLAMPS:
      // Verify both clamps are extended
      if (isClampExtended(POSITION_CLAMP) && isClampExtended(WOOD_SECURE_CLAMP)) {
        cuttingState = EXECUTE_CUT;
      } else {
        // Ensure clamps are extended
        extendClamp(POSITION_CLAMP);
        extendClamp(WOOD_SECURE_CLAMP);
        
        // Wait a brief moment for clamps to engage
        if (Wait(200, &startTime)) {
          cuttingState = EXECUTE_CUT;
        }
      }
      break;
      
    case EXECUTE_CUT:
      // Move cut motor to full travel position
      if (cutMotor.currentPosition() == 0) {
        cutMotor.moveTo(CUT_MOTOR_FULL_TRAVEL);
      }
      
      // Check for wood suction at 0.5 inch into the cut
      if (!woodSuctionChecked && cutMotor.currentPosition() >= CUT_MOTOR_WOOD_SUCTION_CHECK_POS) {
        woodSuctionChecked = true;
        
        if (!isWoodSuctionCorrect()) {
          // Wood suction error detected
          // Serial.println("Wood suction error detected");  // Commented out as specified
          currentError = WOOD_SUCTION_ERROR;
          currentState = ERROR;
          cuttingState = INIT_CUTTING; // Reset for next time
          return;
        }
      }
      
      // Send signal to TA when cut motor reaches the signal position
      if (!signalSentToTA && cutMotor.currentPosition() >= CUT_MOTOR_TA_SIGNAL_POS) {
        // Serial.println("Sending signal to TA");  // Commented out as specified
        digitalWrite(SIGNAL_TO_STAGE_1TO2, HIGH);
        signalSentToTA = true;
      }
      
      // When cut motor completes its travel
      if (cutMotor.distanceToGo() == 0) {
        // Turn off TA signal if it was on
        digitalWrite(SIGNAL_TO_STAGE_1TO2, LOW);
        
        // Check if wood is present for next cut
        cuttingState = CHECK_WOOD;
      }
      break;
      
    case CHECK_WOOD:
      // Determine next state based on wood sensor
      if (checkWoodPresent()) {
        currentState = YESWOOD;
      } else {
        currentState = NOWOOD;
      }
      
      cuttingState = INIT_CUTTING; // Reset for next cycle
      break;
  }
}

// Yes Wood state handler
void YESwood() {
  static enum {
    INIT_YESWOOD,
    RETURN_MOTORS,
    RETRACT_POSITION_CLAMP,
    VERIFY_HOME_POSITION,
    EXTEND_POSITION_CLAMP,
    MOVE_POSITION_MOTOR,
    VERIFY_WOOD_CLAMP,
    YESWOOD_COMPLETE
  } yesWoodState = INIT_YESWOOD;
  
  switch(yesWoodState) {
    case INIT_YESWOOD:
      // Initialize YesWood sequence
      // Serial.println("Wood present - beginning positioning for next cut");  // Commented out as specified
      
      // Set return speeds
      cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
      positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
      
      yesWoodState = RETURN_MOTORS;
      break;
      
    case RETURN_MOTORS:
      // Start both motors moving back to home position
      cutMotor.moveTo(0);
      positionMotor.moveTo(0);
      
      // Check if position motor has moved 0.1 inches toward home
      if (positionMotor.currentPosition() <= (POSITION_MOTOR_FULL_TRAVEL - POSITION_MOTOR_CLAMP_RETRACT_POS)) {
        yesWoodState = RETRACT_POSITION_CLAMP;
      }
      break;
      
    case RETRACT_POSITION_CLAMP:
      // Retract position clamp when motor has moved enough
      retractClamp(POSITION_CLAMP);
      
      // Check if both motors have reached home
      if (cutMotor.distanceToGo() == 0 && positionMotor.distanceToGo() == 0) {
        yesWoodState = VERIFY_HOME_POSITION;
        startTime = 0; // Reset timer for verification wait
      }
      break;
      
    case VERIFY_HOME_POSITION:
      // Wait 50ms then verify cut motor home position switch
      if (Wait(50, &startTime)) {
        if (!isCutMotorAtHome()) {
          // Home position error detected
          // Serial.println("Cut motor home error detected");  // Commented out as specified
          currentError = CUT_MOTOR_HOME_ERROR;
          currentState = ERROR;
          yesWoodState = INIT_YESWOOD; // Reset for next time
          return;
        }
        yesWoodState = EXTEND_POSITION_CLAMP;
      }
      break;
      
    case EXTEND_POSITION_CLAMP:
      // Extend position clamp
      extendClamp(POSITION_CLAMP);
      
      // Wait for clamp to fully extend
      if (Wait(200, &startTime)) {
        yesWoodState = MOVE_POSITION_MOTOR;
      }
      break;
      
    case MOVE_POSITION_MOTOR:
      // Move position motor to full travel position for next cut
      positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
      positionMotor.moveTo(POSITION_MOTOR_FULL_TRAVEL);
      
      // When position motor completes its travel
      if (positionMotor.distanceToGo() == 0) {
        yesWoodState = VERIFY_WOOD_CLAMP;
      }
      break;
      
    case VERIFY_WOOD_CLAMP:
      // Verify wood secure clamp is extended
      if (isClampExtended(WOOD_SECURE_CLAMP)) {
        yesWoodState = YESWOOD_COMPLETE;
      } else {
        extendClamp(WOOD_SECURE_CLAMP);
        // Wait a brief moment for clamp to engage
        if (Wait(200, &startTime)) {
          yesWoodState = YESWOOD_COMPLETE;
        }
      }
      break;
      
    case YESWOOD_COMPLETE:
      // Reset for next cycle
      yesWoodState = INIT_YESWOOD;
      
      // If cycle switch is still on, start a new cutting cycle
      if (isCycleSwitchOn()) {
        currentState = CUTTING;
        signalSentToTA = false;
        woodSuctionChecked = false;
      } else {
        currentState = READY;
      }
      break;
  }
}

// No Wood state handler
void NOwood() {
  static enum {
    INIT_NOWOOD,
    RETRACT_WOOD_CLAMP,
    RETURN_MOTORS,
    RETRACT_POSITION_CLAMP,
    NOWOOD_COMPLETE
  } noWoodState = INIT_NOWOOD;
  
  switch(noWoodState) {
    case INIT_NOWOOD:
      // Initialize NoWood sequence
      // Serial.println("No wood detected - completing cycle");  // Commented out as specified
      
      // Set return speeds
      cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
      positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
      
      noWoodState = RETRACT_WOOD_CLAMP;
      break;
      
    case RETRACT_WOOD_CLAMP:
      // Retract wood secure clamp
      retractClamp(WOOD_SECURE_CLAMP);
      
      // Wait a brief moment for clamp to retract
      if (Wait(200, &startTime)) {
        noWoodState = RETURN_MOTORS;
      }
      break;
      
    case RETURN_MOTORS:
      // Move both motors back to home position
      cutMotor.moveTo(0);
      positionMotor.moveTo(0);
      
      // When both motors complete their travel
      if (cutMotor.distanceToGo() == 0 && positionMotor.distanceToGo() == 0) {
        noWoodState = RETRACT_POSITION_CLAMP;
      }
      break;
      
    case RETRACT_POSITION_CLAMP:
      // Retract position clamp
      retractClamp(POSITION_CLAMP);
      
      // Wait a brief moment for clamp to retract
      if (Wait(200, &startTime)) {
        noWoodState = NOWOOD_COMPLETE;
      }
      break;
      
    case NOWOOD_COMPLETE:
      // Set flag to indicate no-wood cycle is completed
      isNoWoodCycleCompleted = true;
      
      // Reset for next time
      noWoodState = INIT_NOWOOD;
      
      // Return to ready state
      currentState = READY;
      break;
  }
}

// Error handling functions
void handleCutMotorHomingError() {
  static enum {
    INIT_ERROR,
    RETRY_MOVE,
    WAIT_ACK
  } errorState = INIT_ERROR;
  
  static int retryCount = 0;
  
  switch(errorState) {
    case INIT_ERROR:
      // Serial.println("Cut motor homing error");  // Commented out as specified
      // Stop motors
      cutMotor.stop();
      positionMotor.stop();
      errorState = RETRY_MOVE;
      break;
      
    case RETRY_MOVE:
      if (retryCount < 2) {
        // Move cut motor away from home
        cutMotor.moveTo(cutMotor.currentPosition() + CUT_MOTOR_STEPS_PER_INCH);
        if (cutMotor.distanceToGo() == 0) {
          retryCount++;
          errorState = WAIT_ACK;
        }
      } else {
        errorState = WAIT_ACK;
      }
      break;
      
    case WAIT_ACK:
      if (isErrorAcknowledged) {
        // Reset and return to homing
        retryCount = 0;
        hasHomingError = false;
        isErrorAcknowledged = false;
        currentState = HOMING;
        errorState = INIT_ERROR;
      }
      break;
  }
}

void suctionError() {
  static bool waitingForAck = false;
  
  if (!waitingForAck) {
    // Serial.println("Suction error detected");  // Commented out as specified
    // Stop motors
    cutMotor.stop();
    positionMotor.stop();
    waitingForAck = true;
  }
  
  if (isErrorAcknowledged) {
    // Reset and return to homing
    waitingForAck = false;
    hasWoodSuctionError = false;
    isErrorAcknowledged = false;
    currentState = HOMING;
  }
}
