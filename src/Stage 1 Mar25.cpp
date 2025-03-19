// Stage 1 Wood Cutting Machine
// Controls two stepper motors and two pneumatic clamps to precisely cut wood boards

#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

<<<<<<< HEAD
// -------------------- SYSTEM STATES --------------------
=======
// ===== PIN DEFINITIONS =====

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 48       // Controls the pulse signal to the cut motor driver - moves the sliding table saw carriage
#define CUT_MOTOR_DIR_PIN 47         // Controls the direction of the cut motor (forward/backward movement)

#define POSITION_MOTOR_PULSE_PIN 21  // Controls the pulse signal to the position motor driver - feeds wood forward for next cut or retrieves the wood during the NOwood operation.
#define POSITION_MOTOR_DIR_PIN 20    // Controls the direction of the position motor (forward/backward movement)

// Switch and Sensor Pin Definitions (Left side inputs)
#define CUT_MOTOR_HOMING_SWITCH 10      // Limit switch that detects when cut motor is at home position
#define POSITION_MOTOR_HOMING_SWITCH 9 // Limit switch that detects when position motor is at home position
#define RELOAD_SWITCH 14                  // Manual switch to enter reload mode - disengages clamps for material loading
#define WOOD_SENSOR 11                    // Sensor that detects if wood is present (LOW when wood is detected)

// Switch and Sensor Pin Definitions (Right side)
#define CYCLE_SWITCH 13          // Switch that initiates the cutting cycle when activated. This can be left on for continuous operation. The system will cut the wood until no wood is detecteed.
#define WAS_WOOD_SUCTIONED_SENSOR 8    // Sensor that checks if wood was properly suctioned during cutting (error detection)

// Clamp Pin Definitions
#define POSITION_CLAMP 18           // Controls the pneumatic clamp that holds the positioning mechanism (LOW = extended)
#define WOOD_SECURE_CLAMP 17        // Controls the pneumatic clamp that secures the wood piece (LOW = extended)

// LED Pin Definitions
#define RED_LED 7    // Error indicator LED - blinks during error conditions
#define YELLOW_LED 6 // Operation in progress indicator - on during cutting or when in reload mode
#define GREEN_LED 16   // Ready indicator - on when system is ready to begin a cutting cycle
#define BLUE_LED 15    // Setup/special mode indicator - on during startup or when no wood present

// Signal Output
#define SIGNAL_TO_STAGE_1TO2 19    // Output signal to the next stage in the process (Stage 2) when cutting is complete

// ===== CONSTANTS =====
// System States
>>>>>>> df31cdf6dd01455d372c42309681ede7badaf81b
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

// -------------------- ERROR TYPES --------------------
enum ErrorType {
  NONE,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR
};

// -------------------- PIN ASSIGNMENTS --------------------
// Motor Control Pins
const int CUT_MOTOR_PULSE_PIN = 39;
const int CUT_MOTOR_DIR_PIN = 38;
const int POSITION_MOTOR_PULSE_PIN = 1;
const int POSITION_MOTOR_DIR_PIN = 2;

// Switch and Sensor Pins
const int CUT_MOTOR_POS_SWITCH_PIN = 10;
const int POSITION_MOTOR_POS_SWITCH_PIN = 9;
const int RELOAD_SWITCH_PIN = 14;
const int CYCLE_SWITCH_PIN = 13;
const int WOOD_SENSOR_PIN = 11;
const int WOOD_SUCTION_SENSOR_PIN = 12;

// Clamp Pins (LOW = Extended/Engaged, HIGH = Retracted)
const int POSITION_CLAMP_PIN = 18;
const int WOOD_SECURE_CLAMP_PIN = 17;

// LED Pins
const int RED_LED_PIN = 16;
const int YELLOW_LED_PIN = 15;
const int GREEN_LED_PIN = 7;
const int BLUE_LED_PIN = 6;

// Signal Output
const int SIGNAL_TO_TA_PIN = 19;

// -------------------- MOTOR PARAMETERS --------------------
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

// -------------------- POSITION CONSTANTS --------------------
const long CUT_MOTOR_FULL_TRAVEL = CUT_MOTOR_STEPS_PER_INCH * CUT_MOTOR_TRAVEL_DISTANCE;
const long POSITION_MOTOR_FULL_TRAVEL = POSITION_MOTOR_STEPS_PER_INCH * POSITION_MOTOR_TRAVEL_DISTANCE;
const long CUT_MOTOR_WOOD_SUCTION_CHECK_POS = CUT_MOTOR_STEPS_PER_INCH * 0.5;  // 0.5 inch
const long CUT_MOTOR_TA_SIGNAL_POS = CUT_MOTOR_STEPS_PER_INCH * 7.2;           // 7.2 inches
const long POSITION_MOTOR_CLAMP_RETRACT_POS = POSITION_MOTOR_STEPS_PER_INCH * 0.1; // 0.1 inch

// -------------------- SYSTEM VARIABLES --------------------
SystemState currentState = STARTUP;
ErrorType currentError = NONE;
bool isNoWoodCycleCompleted = false;
bool signalSentToTA = false;
bool woodSuctionChecked = false;

// -------------------- TIMING VARIABLES --------------------
unsigned long startTime = 0;
unsigned long blinkStartTime = 0;
unsigned long errorBlinkStartTime = 0;

// -------------------- DEBOUNCE OBJECTS --------------------
Bounce cutMotorPosSwitchDebounce = Bounce();
Bounce positionMotorPosSwitchDebounce = Bounce();
Bounce reloadSwitchDebounce = Bounce();
Bounce cycleSwitchDebounce = Bounce();
Bounce woodSensorDebounce = Bounce();
Bounce woodSuctionSensorDebounce = Bounce();

<<<<<<< HEAD
// -------------------- STEPPER MOTOR OBJECTS --------------------
// 1 = step interface type (Step/Dir pins)
AccelStepper cutMotor(1, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(1, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);
=======
// System Flags
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
bool isNoWoodCycleCompleted = false;  // Flag to track if we just completed a NOwood operation

// Timers
unsigned long currentTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastErrorBlinkTime = 0;
unsigned long errorStartTime = 0;
unsigned long positionMoveStartTime = 0;
unsigned long delayStartTime = 0;

// LED States
bool blinkState = false;
bool errorBlinkState = false;

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
void NOwood();  // Renamed from noWoodOperation
void handleCutMotorHomingError();
void suctionError();

// Utility Functions
bool Wait(unsigned long delayTime, unsigned long* startTimePtr);

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
  pinMode(RELOAD_SWITCH, INPUT);                       // External pull-down
  pinMode(CYCLE_SWITCH, INPUT);                  // External pull-down
  pinMode(WOOD_SENSOR, INPUT_PULLUP);                  // Active LOW (LOW = wood present)
  pinMode(WAS_WOOD_SUCTIONED_SENSOR, INPUT);           // External pull-down
  
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
  cutMotor.setMaxSpeed(CUTTINGSPEED);
  cutMotor.setAcceleration(CUT_ACCELERATION);
  positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_ACCELERATION);
  
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

void readSwitchStates () {
  // Update all debounced switches
  cutHomingSensor.update();
  positionHomingSensor.update();
  reloadSwitch.update();
  cycleSwitch.update();
  
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
>>>>>>> df31cdf6dd01455d372c42309681ede7badaf81b

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

// -------------------- FUNCTION PROTOTYPES --------------------
void setupSystem();
void updateSwitches();
void updateState();
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
bool isWoodPresent();
bool isWoodSuctionCorrect();
bool isCycleSwitchOn();
bool isReloadSwitchOn();

// -------------------- SETUP FUNCTION --------------------
void setup() {
  // Initialize serial communication
  // Serial.begin(115200);  // Commented out as specified
  
  // Call setup function
  setupSystem();
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // Update all switch readings with debounce
  updateSwitches();
  
  // Run stepper motors
  cutMotor.run();
  positionMotor.run();
  
  // Handle the current state
  updateState();
}

// -------------------- SETUP SYSTEM --------------------
void setupSystem() {
  // Setup pin modes
  pinMode(CUT_MOTOR_POS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(POSITION_MOTOR_POS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RELOAD_SWITCH_PIN, INPUT_PULLUP);
  pinMode(CYCLE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(WOOD_SENSOR_PIN, INPUT_PULLUP);
  pinMode(WOOD_SUCTION_SENSOR_PIN, INPUT_PULLUP);
  
  pinMode(POSITION_CLAMP_PIN, OUTPUT);
  pinMode(WOOD_SECURE_CLAMP_PIN, OUTPUT);
  
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  pinMode(SIGNAL_TO_TA_PIN, OUTPUT);
  
  // Initialize switch debouncing
  cutMotorPosSwitchDebounce.attach(CUT_MOTOR_POS_SWITCH_PIN);
  positionMotorPosSwitchDebounce.attach(POSITION_MOTOR_POS_SWITCH_PIN);
  reloadSwitchDebounce.attach(RELOAD_SWITCH_PIN);
  cycleSwitchDebounce.attach(CYCLE_SWITCH_PIN);
  woodSensorDebounce.attach(WOOD_SENSOR_PIN);
  woodSuctionSensorDebounce.attach(WOOD_SUCTION_SENSOR_PIN);
  
  // Set 20ms debounce time for all switches
  cutMotorPosSwitchDebounce.interval(20);
  positionMotorPosSwitchDebounce.interval(20);
  reloadSwitchDebounce.interval(20);
  cycleSwitchDebounce.interval(20);
  woodSensorDebounce.interval(20);
  woodSuctionSensorDebounce.interval(20);
  
  // Initialize outputs
  digitalWrite(SIGNAL_TO_TA_PIN, LOW); // Initialize transfer arm signal as LOW
  retractClamp(POSITION_CLAMP_PIN);    // Initialize clamps as retracted
  retractClamp(WOOD_SECURE_CLAMP_PIN);
  
  // Turn off all LEDs initially
  setLEDs(false, false, false, false);
  
  // Set motor parameters
  cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
  
  positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
  
  // Set initial state
  currentState = STARTUP;
  
  // Check if cycle switch is ON at startup (safety feature)
  cycleSwitchDebounce.update();
  if (isCycleSwitchOn()) {
    // Safety feature: Wait until cycle switch is turned OFF before proceeding
    while (isCycleSwitchOn()) {
      cycleSwitchDebounce.update();
      blinkLED(RED_LED_PIN, 200); // Fast blink to indicate error
      delay(10); // Small delay for switch reading
    }
  }
}

// -------------------- UPDATE SWITCHES --------------------
void updateSwitches() {
  cutMotorPosSwitchDebounce.update();
  positionMotorPosSwitchDebounce.update();
  reloadSwitchDebounce.update();
  cycleSwitchDebounce.update();
  woodSensorDebounce.update();
  woodSuctionSensorDebounce.update();
}

// -------------------- UPDATE STATE --------------------
void updateState() {
  switch(currentState) {
    case STARTUP:
      handleStartup();
      break;
    case HOMING:
      handleHoming();
      break;
    case READY:
      handleReady();
      break;
    case RELOAD:
      handleReload();
      break;
    case CUTTING:
      handleCutting();
      break;
    case YESWOOD:
      handleYesWood();
      break;
    case NOWOOD:
      handleNoWood();
      break;
    case ERROR:
      handleError();
      break;
    default:
      // Should never get here, but reset to startup if it happens
      currentState = STARTUP;
      break;
  }
}

// -------------------- STATE HANDLER: STARTUP --------------------
void handleStartup() {
  // Turn on blue LED during startup
  setLEDs(false, false, false, true);
  
  // Wait a brief moment to ensure system is initialized
  if (Wait(1000, &startTime)) {
    // Move directly to homing state
    currentState = HOMING;
    startTime = 0; // Reset timer for next state
  }
}

// -------------------- STATE HANDLER: HOMING --------------------
void handleHoming() {
  static enum {
    HOMING_START,
    MOVE_FROM_SWITCHES_IF_NEEDED,
    HOME_CUT_MOTOR,
    HOME_POSITION_MOTOR,
    MOVE_POSITION_TO_OPERATING,
    HOMING_COMPLETE
  } homingState = HOMING_START;
  
  // Blink blue LED during homing
  blinkLED(BLUE_LED_PIN, 500);
  
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
      extendClamp(POSITION_CLAMP_PIN);
      extendClamp(WOOD_SECURE_CLAMP_PIN);
      
      // Transition to READY state
      // Serial.println("Homing complete");  // Commented out as specified
      homingState = HOMING_START; // Reset for next time
      currentState = READY;
      break;
  }
}

// -------------------- STATE HANDLER: READY --------------------
void handleReady() {
  // Set LEDs - Green ON, others OFF
  setLEDs(false, false, true, false);
  
  // Check reload switch
  if (isReloadSwitchOn()) {
    currentState = RELOAD;
    return;
  }
  
  // Check cycle switch and if wood is present to start cycle
  if (isCycleSwitchOn() && isWoodPresent()) {
    // Reset cycle variables
    signalSentToTA = false;
    woodSuctionChecked = false;
    currentState = CUTTING;
  }
}

// -------------------- STATE HANDLER: RELOAD --------------------
void handleReload() {
  // Set LEDs - Green and Blue ON
  setLEDs(false, true, true, true);
  
  // Retract both clamps to allow wood loading/unloading
  retractClamp(POSITION_CLAMP_PIN);
  retractClamp(WOOD_SECURE_CLAMP_PIN);
  
  // Return to READY state when reload switch is released
  if (!isReloadSwitchOn()) {
    // Re-extend both clamps before returning to ready
    extendClamp(POSITION_CLAMP_PIN);
    extendClamp(WOOD_SECURE_CLAMP_PIN);
    currentState = READY;
  }
}

// -------------------- STATE HANDLER: CUTTING --------------------
void handleCutting() {
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
      if (isClampExtended(POSITION_CLAMP_PIN) && isClampExtended(WOOD_SECURE_CLAMP_PIN)) {
        cuttingState = EXECUTE_CUT;
      } else {
        // Ensure clamps are extended
        extendClamp(POSITION_CLAMP_PIN);
        extendClamp(WOOD_SECURE_CLAMP_PIN);
        
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
        digitalWrite(SIGNAL_TO_TA_PIN, HIGH);
        signalSentToTA = true;
      }
      
      // When cut motor completes its travel
      if (cutMotor.distanceToGo() == 0) {
        // Turn off TA signal if it was on
        digitalWrite(SIGNAL_TO_TA_PIN, LOW);
        
        // Check if wood is present for next cut
        cuttingState = CHECK_WOOD;
      }
      break;
      
    case CHECK_WOOD:
      // Determine next state based on wood sensor
      if (isWoodPresent()) {
        currentState = YESWOOD;
      } else {
        currentState = NOWOOD;
      }
      
      cuttingState = INIT_CUTTING; // Reset for next cycle
      break;
  }
}

// -------------------- STATE HANDLER: YESWOOD --------------------
void handleYesWood() {
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
      retractClamp(POSITION_CLAMP_PIN);
      
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
      extendClamp(POSITION_CLAMP_PIN);
      
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
      if (isClampExtended(WOOD_SECURE_CLAMP_PIN)) {
        yesWoodState = YESWOOD_COMPLETE;
      } else {
        extendClamp(WOOD_SECURE_CLAMP_PIN);
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

// -------------------- STATE HANDLER: NOWOOD --------------------
void handleNoWood() {
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
      retractClamp(WOOD_SECURE_CLAMP_PIN);
      
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
      retractClamp(POSITION_CLAMP_PIN);
      
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

// -------------------- STATE HANDLER: ERROR --------------------
void handleError() {
  static enum {
    INIT_ERROR,
    DISPLAY_ERROR,
    HANDLE_CUT_HOME_ERROR,
    WAIT_FOR_ACKNOWLEDGMENT
  } errorState = INIT_ERROR;
  
  static int errorRetryCount = 0;
  
  switch(errorState) {
    case INIT_ERROR:
      // Initialize error handling
      // Serial.print("Error state: ");  // Commented out as specified
      
      // Stop motors
      cutMotor.stop();
      positionMotor.stop();
      
      // Turn off TA signal if it was on
      digitalWrite(SIGNAL_TO_TA_PIN, LOW);
      
      errorState = DISPLAY_ERROR;
      break;
      
    case DISPLAY_ERROR:
      // Display error via LED pattern
      blinkErrorPattern();
      
      if (currentError == CUT_MOTOR_HOME_ERROR) {
        errorState = HANDLE_CUT_HOME_ERROR;
      } else {
        errorState = WAIT_FOR_ACKNOWLEDGMENT;
      }
      break;
      
    case HANDLE_CUT_HOME_ERROR:
      // Attempt to recover from cut motor home error
      if (errorRetryCount < 2) { // Try two recovery attempts
        // Move cut motor forward 1 inch
        cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
        cutMotor.moveTo(cutMotor.currentPosition() + CUT_MOTOR_STEPS_PER_INCH);
        
        if (cutMotor.distanceToGo() == 0) {
          // Retry homing after moving forward
          errorRetryCount++;
          errorState = WAIT_FOR_ACKNOWLEDGMENT;
        }
      } else {
        // After two attempts, wait for user acknowledgment
        errorState = WAIT_FOR_ACKNOWLEDGMENT;
      }
      break;
      
    case WAIT_FOR_ACKNOWLEDGMENT:
      // Wait for reload switch to be pressed as acknowledgment
      if (isReloadSwitchOn()) {
        // Reset error state
        errorRetryCount = 0;
        currentError = NONE;
        errorState = INIT_ERROR;
        
        // Return to homing state to re-establish system
        currentState = HOMING;
      }
      break;
  }
}

// -------------------- LED CONTROL FUNCTIONS --------------------
void setLEDs(bool red, bool yellow, bool green, bool blue) {
  digitalWrite(RED_LED_PIN, red ? HIGH : LOW);
  digitalWrite(YELLOW_LED_PIN, yellow ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, green ? HIGH : LOW);
  digitalWrite(BLUE_LED_PIN, blue ? HIGH : LOW);
}

void blinkLED(int pin, int interval) {
  unsigned long currentTime = millis();
  
  if (currentTime - blinkStartTime >= interval) {
    digitalWrite(pin, !digitalRead(pin)); // Toggle LED state
    blinkStartTime = currentTime;
  }
}

void blinkErrorPattern() {
  // Different patterns based on error type
  int onTime = 200;  // Time LED is on (ms)
  int offTime = 200; // Time LED is off between flashes (ms)
  int pauseTime = 1000; // Time between patterns (ms)
  int numFlashes = 0;
  
  switch(currentError) {
    case WOOD_SUCTION_ERROR:
      numFlashes = 3;
      break;
    case CUT_MOTOR_HOME_ERROR:
      numFlashes = 2;
      break;
    default:
      numFlashes = 1; // Default for unknown errors
      break;
  }
  
  static enum {
    START_PATTERN,
    LED_ON,
    LED_OFF,
    PATTERN_PAUSE
  } patternState = START_PATTERN;
  
  static int flashCount = 0;
  
  switch(patternState) {
    case START_PATTERN:
      flashCount = 0;
      digitalWrite(RED_LED_PIN, LOW);
      errorBlinkStartTime = millis();
      patternState = LED_ON;
      break;
      
    case LED_ON:
      digitalWrite(RED_LED_PIN, HIGH);
      if (millis() - errorBlinkStartTime >= onTime) {
        errorBlinkStartTime = millis();
        patternState = LED_OFF;
      }
      break;
      
    case LED_OFF:
      digitalWrite(RED_LED_PIN, LOW);
      if (millis() - errorBlinkStartTime >= offTime) {
        flashCount++;
        errorBlinkStartTime = millis();
        
        if (flashCount >= numFlashes) {
          patternState = PATTERN_PAUSE;
        } else {
          patternState = LED_ON;
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

// -------------------- CLAMP CONTROL FUNCTIONS --------------------
void extendClamp(int clampPin) {
  digitalWrite(clampPin, LOW); // LOW = Extended/Engaged
}

void retractClamp(int clampPin) {
  digitalWrite(clampPin, HIGH); // HIGH = Retracted
}

bool isClampExtended(int clampPin) {
  return digitalRead(clampPin) == LOW;
}

bool isClampRetracted(int clampPin) {
  return digitalRead(clampPin) == HIGH;
}

// -------------------- SENSOR/SWITCH CHECK FUNCTIONS --------------------
bool isCutMotorAtHome() {
  return cutMotorPosSwitchDebounce.read() == HIGH; // Switch is HIGH when triggered
}

bool isPositionMotorAtHome() {
  return positionMotorPosSwitchDebounce.read() == HIGH; // Switch is HIGH when triggered
}

bool isWoodPresent() {
  return woodSensorDebounce.read() == LOW; // LOW when wood is present
}

bool isWoodSuctionCorrect() {
  return woodSuctionSensorDebounce.read() == LOW; // LOW when suction is correct
}

bool isCycleSwitchOn() {
  return cycleSwitchDebounce.read() == HIGH; // HIGH when switch is ON
}

bool isReloadSwitchOn() {
  return reloadSwitchDebounce.read() == HIGH; // HIGH when switch is ON
}
