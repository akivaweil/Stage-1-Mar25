#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

// ===== PIN DEFINITIONS =====
// ESP32 Breakout Board Layout:
// Left side (top to bottom): 34, 35, 32, 33, 25, 26, 27, 14, 12, 13
// Right side (top to bottom): 23, 22, 21, 19, 18, 5, 4, 0, 2, 15

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 22      // Right side, top
#define CUT_MOTOR_DIR_PIN 23        // Right side, top
#define POSITION_MOTOR_PULSE_PIN 32 // Left side, upper section
#define POSITION_MOTOR_DIR_PIN 33   // Left side, upper section

// Switch and Sensor Pin Definitions
#define CUT_MOTOR_HOMING_SENSOR 25      // Left side, middle section
#define POSITION_MOTOR_HOMING_SENSOR 27 // Left side, middle section
#define RELOAD_SWITCH 14                  // Left side, near bottom
#define START_CYCLE_SWITCH 18             // Right side, middle section
#define WOOD_SENSOR 35                    // Left side, top
#define WAS_WOOD_SUCTIONED_SENSOR 5       // Right side, middle section

// Clamp Pin Definitions
#define POSITION_CLAMP 13           // Left side, bottom
#define WOOD_SECURE_CLAMP 15        // Right side, bottom

// LED Pin Definitions
#define RED_LED 26    // Error LED - Left side, middle section
#define YELLOW_LED 21  // Busy/Reload LED - Right side, upper section
#define GREEN_LED 4    // Ready LED - Right side, lower section
#define BLUE_LED 2     // Setup/No-Wood LED - Right side, near bottom

// Signal Output
#define SIGNAL_TO_TA_PIN 19    // Right side, upper middle section

// ===== CONSTANTS =====
// System States
enum SystemState {
  STARTUP,
  HOMING,
  READY,
  CUTTING,
  YESWOOD,  // Renamed from POSITIONING
  ERROR,
  ERROR_RESET
};

// Motor Configuration
#define CUT_MOTOR_STEPS_PER_INCH 63.5  // 200 steps/rev with 40T pulley on 2GT belt (80mm/rev)
#define POSITION_MOTOR_STEPS_PER_INCH 1000
#define CUT_TRAVEL_DISTANCE 7.0 // inches
#define POSITION_TRAVEL_DISTANCE 3.45 // inches
#define CUT_HOMING_DIRECTION -1
#define POSITION_HOMING_DIRECTION -1

// Speed and Acceleration Settings
#define CUT_NORMAL_SPEED 120
#define CUT_RETURN_SPEED 2500
#define CUT_ACCELERATION 3000
#define CUT_HOMING_SPEED 300
#define CUT_APPROACH_SPEED (CUT_HOMING_SPEED * 0.85) // 15% slower for sensor approach
#define POSITION_NORMAL_SPEED 50000
#define POSITION_RETURN_SPEED 50000
#define POSITION_ACCELERATION 50000
#define POSITION_RETURN_ACCELERATION 50000
#define POSITION_HOMING_SPEED 2000 // Slower speed for homing operations

// Timing Parameters
#define SIGNAL_DEBOUNCE_INTERVAL 10
#define BLINK_INTERVAL 500
#define ERROR_BLINK_INTERVAL 250

// ===== GLOBAL VARIABLES =====
// System State
SystemState currentState = STARTUP;

// Motor Objects
AccelStepper cutMotor(AccelStepper::DRIVER, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(AccelStepper::DRIVER, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

// Debouncer Objects
Bounce cutHomingSensor = Bounce();
Bounce positionHomingSensor = Bounce();
Bounce reloadSwitch = Bounce();
Bounce startCycleSwitch = Bounce();

// System Flags
bool isHomed = false;
bool isReloadMode = false;
bool woodPresent = false;
bool woodSuctionError = false;
bool signalSent = false;
bool errorAcknowledged = false;
bool cuttingCycleInProgress = false;
bool continuousModeActive = false;
bool startSwitchSafe = false;

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
void CUTmovement();
void YESwood();  // Renamed from positioningOperation
void NOwood();  // Renamed from noWoodOperation
void handleErrorState();
void resetFromError();

// Utility Functions
bool Wait(unsigned long delayTime, unsigned long* startTimePtr);

void setup() {
  // Initialize the system
  initializeSystem();
  
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("Stage 1 system starting...");
  
  // Start in STARTUP state
  currentState = STARTUP;
  
  // Check if start switch is already ON at startup
  startCycleSwitch.update();
  if (startCycleSwitch.read() == HIGH) {
    startSwitchSafe = false;
    Serial.println("WARNING: Start switch is ON at startup");
  } else {
    startSwitchSafe = true;
  }
  
  // Brief delay before starting homing
  delay(10);
}

void initializeSystem() {
  // Configure motor pins
  pinMode(CUT_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(CUT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_DIR_PIN, OUTPUT);
  
  // Configure switch pins
  pinMode(CUT_MOTOR_HOMING_SENSOR, INPUT_PULLUP);
  pinMode(POSITION_MOTOR_HOMING_SENSOR, INPUT_PULLUP);
  pinMode(RELOAD_SWITCH, INPUT);
  pinMode(START_CYCLE_SWITCH, INPUT);
  
  // Configure sensor pins
  pinMode(WOOD_SENSOR, INPUT_PULLUP);          // Active LOW (LOW = wood present)
  pinMode(WAS_WOOD_SUCTIONED_SENSOR, INPUT);   // For checking if wood was suctioned
  
  // Configure clamp pins - LOW = engaged (extended), HIGH = disengaged (retracted)
  pinMode(POSITION_CLAMP, OUTPUT);
  pinMode(WOOD_SECURE_CLAMP, OUTPUT);
  digitalWrite(POSITION_CLAMP, LOW);       // Start with position clamp engaged
  digitalWrite(WOOD_SECURE_CLAMP, LOW);    // Start with wood secure clamp engaged
  
  // Configure LED pins
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  // All LEDs off to start
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  
  // Turn on blue LED during startup
  digitalWrite(BLUE_LED, HIGH);
  
  // Configure signal output pin
  pinMode(SIGNAL_TO_TA_PIN, OUTPUT);
  digitalWrite(SIGNAL_TO_TA_PIN, LOW); // Initialize as LOW (inactive)
  
  // Set up debouncing for switches
  cutHomingSensor.attach(CUT_MOTOR_HOMING_SENSOR);
  cutHomingSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  
  positionHomingSensor.attach(POSITION_MOTOR_HOMING_SENSOR);
  positionHomingSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  
  reloadSwitch.attach(RELOAD_SWITCH);
  reloadSwitch.interval(SIGNAL_DEBOUNCE_INTERVAL);
  
  startCycleSwitch.attach(START_CYCLE_SWITCH);
  startCycleSwitch.interval(SIGNAL_DEBOUNCE_INTERVAL);
  
  // Configure motors
  cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_ACCELERATION);
  
  positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_ACCELERATION);
  
  // Initially set motors to use position 0
  cutMotor.setCurrentPosition(0);
  positionMotor.setCurrentPosition(0);
}

void readSwitchStates () {
  // Update all debounced switches
  cutHomingSensor.update();
  positionHomingSensor.update();
  reloadSwitch.update();
  startCycleSwitch.update();
  
  // Read wood sensor (active LOW)
  woodPresent = (digitalRead(WOOD_SENSOR) == LOW);
}

void loop() {
  // Read all switch states
  readSwitchStates();
  
  // Handle start switch safety check
  if (!startSwitchSafe && startCycleSwitch.fell()) {
    // Start switch was turned OFF after being ON during startup
    startSwitchSafe = true;
  }
  
  // Handle the reload switch state when in READY state
  if (currentState == READY) {
    // Check current state of reload switch (HIGH = ON with pull-down resistor)
    bool reloadSwitchOn = reloadSwitch.read() == HIGH;
    
    if (reloadSwitchOn && !isReloadMode) {
      // Enter reload mode
      isReloadMode = true;
      digitalWrite(POSITION_CLAMP, HIGH); // Disengage position clamp
      digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Disengage wood secure clamp
      digitalWrite(YELLOW_LED, HIGH);     // Turn on yellow LED for reload mode
    } else if (!reloadSwitchOn && isReloadMode) {
      // Exit reload mode
      isReloadMode = false;
      digitalWrite(POSITION_CLAMP, LOW);   // Re-engage position clamp
      digitalWrite(WOOD_SECURE_CLAMP, LOW); // Re-engage wood secure clamp
      digitalWrite(YELLOW_LED, LOW);       // Turn off yellow LED
    }
  }
  
  // Handle error acknowledgment separately
  if (reloadSwitch.rose() && currentState == ERROR) {
    currentState = ERROR_RESET;
    errorAcknowledged = true;
  }
  
  // Check for continuous mode activation/deactivation
  bool startSwitchOn = startCycleSwitch.read() == HIGH;
  if (startSwitchOn != continuousModeActive && startSwitchSafe) {
    continuousModeActive = startSwitchOn;
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
      // System is ready for operation
      if (!isReloadMode) {
        // Solid green LED to indicate ready
        digitalWrite(GREEN_LED, HIGH);
        
        // Start a new cycle if:
        // 1. Start switch was just flipped ON (rising edge), OR
        // 2. Continuous mode is active AND we're not already in a cutting cycle
        // AND the start switch is safe to use
        if (((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress)) 
            && !woodSuctionError) && startSwitchSafe) {
          // Turn off ready LED, turn on busy LED
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(YELLOW_LED, HIGH);
          digitalWrite(BLUE_LED, LOW);  // Ensure blue LED is off at start of cutting cycle
          
          // Set flag to indicate cycle in progress
          cuttingCycleInProgress = true;
          
          // Always enter cutting state, regardless of wood presence
          currentState = CUTTING;
          // Configure cut motor for cutting speed
          cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
          cutMotor.setAcceleration(CUT_ACCELERATION);
          // Ensure clamps are engaged
          digitalWrite(POSITION_CLAMP, LOW);
          digitalWrite(WOOD_SECURE_CLAMP, LOW);
          
          // Store wood presence for later use
          if (!woodPresent) {
            digitalWrite(BLUE_LED, HIGH); // Blue LED on for no-wood mode
          }
        }
      }
      break;
      
    case CUTTING:
      CUTmovement();
      break;
      
    case YESWOOD:
      YESwood();
      break;
      
    case ERROR:
      handleErrorState();
      break;
      
    case ERROR_RESET:
      resetFromError();
      break;
  }
  
  // Run the motors (non-blocking)
  cutMotor.run();
  positionMotor.run();
}

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

void homingSequence() {
  static bool cutMotorHomed = false;
  static bool positionMotorHomed = false;
  static bool positionMotorMoved = false;
  static unsigned long homingStartTime = 0;
  static unsigned long blinkTimer = 0;
  
  // Blink blue LED to indicate homing
  if (millis() - blinkTimer > BLINK_INTERVAL) {
    blinkState = !blinkState;
    digitalWrite(BLUE_LED, blinkState);
    blinkTimer = millis();
  }
  
  // Initialize homing start time if not set
  if (homingStartTime == 0) {
    homingStartTime = millis();
  }
  
  // Home cut motor first
  if (!cutMotorHomed) {
    // If the cut motor homing sensor is already active, move away first
    if (cutHomingSensor.read() == HIGH) {
      // Use full speed when moving away from the sensor
      cutMotor.setMaxSpeed(CUT_HOMING_SPEED);
      cutMotor.moveTo(10 * CUT_MOTOR_STEPS_PER_INCH); // Move slightly away from sensor
      if (cutMotor.distanceToGo() == 0) {
        // Now move back to find the sensor at a slower approach speed
        cutMotor.setSpeed(CUT_APPROACH_SPEED * CUT_HOMING_DIRECTION);
        cutMotor.moveTo(-10000); // Move toward sensor (will stop when sensor activated)
      }
    } else {
      // Move toward home sensor at slower approach speed
      cutMotor.setSpeed(CUT_APPROACH_SPEED * CUT_HOMING_DIRECTION);
      cutMotor.moveTo(-10000); // Large number in homing direction
    }
    
    // Check if we hit the sensor
    if (cutHomingSensor.read() == HIGH) {
      cutMotor.stop();
      cutMotor.setCurrentPosition(0);
      cutMotorHomed = true;
    }
  } else if (!positionMotorHomed) {
    // Home position motor
    digitalWrite(POSITION_CLAMP, HIGH); // Disengage position clamp for homing
    
    // If the position motor homing sensor is already active, move away first
    if (positionHomingSensor.read() == HIGH) {
      positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
      positionMotor.moveTo(100 * POSITION_MOTOR_STEPS_PER_INCH); // Move slightly away
      if (positionMotor.distanceToGo() == 0) {
        // Now move back to find the sensor
        positionMotor.setSpeed(POSITION_HOMING_SPEED * POSITION_HOMING_DIRECTION);
        positionMotor.moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH); // Move toward sensor
      }
    } else {
      // Move toward home sensor
      positionMotor.setSpeed(POSITION_HOMING_SPEED * POSITION_HOMING_DIRECTION);
      positionMotor.moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH);
    }
    
    // Check if we hit the sensor
    if (positionHomingSensor.read() == HIGH) {
      positionMotor.stop();
      // Set current position to -1 inch
      positionMotor.setCurrentPosition(-1 * POSITION_MOTOR_STEPS_PER_INCH);
      positionMotorHomed = true;
      
      // Set back to normal speed after homing is complete
      positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
    }
  } else if (!positionMotorMoved) {
    // Move position motor to 3.45 inches from home position
    positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
    
    if (positionMotor.distanceToGo() == 0) {
      positionMotorMoved = true;
      digitalWrite(POSITION_CLAMP, LOW); // Re-engage position clamp
    }
  } else {
    // Homing complete
    cutMotorHomed = false;
    positionMotorHomed = false;
    positionMotorMoved = false;
    homingStartTime = 0;
    
    // Set system as homed
    isHomed = true;
    
    // Turn off blue LED, turn on green LED
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    
    // Transition to READY state
    currentState = READY;
  }
}

void CUTmovement() {
  // Cutting operation:
  // 1. Ensure both clamps are engaged
  // 2. Move cut motor to cutting position
  // 3. Signal TA when complete (active HIGH)
  // 4. Transition to YESwood or NOwood based on wood presence
  
  static int stage = 0;
  static unsigned long clampTimer = 0;
  
  // Step 0: Ensure both clamps are engaged before starting cut movement
  if (stage == 0) {
    // Engage both clamps
    digitalWrite(POSITION_CLAMP, LOW);      // Engage position clamp
    digitalWrite(WOOD_SECURE_CLAMP, LOW);   // Engage wood secure clamp
    
    // Wait for clamps to fully engage
    if (Wait(1, &clampTimer)) {
      stage = 1;  // Move to next stage
    }
    return;
  }
  
  // Step 1: Start the cut motor movement
  if (stage == 1) {
    // Set cut motor parameters and start movement
    cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
    
    // Move to cutting position
    long targetPosition = CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH;
    cutMotor.moveTo(targetPosition);
    
    // Move to next stage
    stage = 2;
    return;
  }
  
  // Step 2: Wait for cutting motion to complete
  if (stage == 2) {
    if (cutMotor.distanceToGo() == 0) {
      // Re-read the wood sensor after cut move finishes
      woodPresent = (digitalRead(WOOD_SENSOR) == LOW); // LOW indicates wood present
      
      // Send signal to TA (Transfer Arm) regardless of wood presence
      digitalWrite(SIGNAL_TO_TA_PIN, HIGH);
      signalSent = true;
      
      // Reset stage for next cycle
      stage = 0;
      
      // Wait a moment for the signal to be received
      static unsigned long signalTimer = 0;
      if (Wait(200, &signalTimer)) {
        // Reset signal (LOW)
        digitalWrite(SIGNAL_TO_TA_PIN, LOW);
        
        if (woodPresent) {
          // Wood present, move to YESwood state
          currentState = YESWOOD;
        } else {
          // No wood present, call NOwood() function
          NOwood();
        }
      }
    }
    return;
  }
}

void YESwood() {
  // Step sequence:
  // 0: Initialize - Move back 0.1 inches with position clamp engaged, retract wood secure clamp, and start cut motor home
  // 1: Wait for 0.1 inch movement to complete, then disengage position clamp and re-engage wood secure clamp
  // 2: Return position motor to home position
  // 3: Wait for position motor to reach home, engage position clamp
  // 4: Verify cut motor has properly returned home by checking homing sensor
  // 5: Move position motor to final position
  // 6: Wait for position motor to reach final position
  // 7: Complete operation
  
  static int stage = 0;
  static long currentPos;
  static bool positionClampEngaged = false;
  static unsigned long clampTimer = 0;
  
  // Always run both motors to ensure they continue moving
  positionMotor.run();
  cutMotor.run();
  
  // Step 0: Initialize - Move back 0.1 inches with position clamp engaged and start cut motor home
  if (stage == 0) {
    // Keep position clamp engaged during initial movement
    digitalWrite(POSITION_CLAMP, LOW);
    
    // Retract wood secure clamp
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    
    // Move 0.1 inches back from current position
    currentPos = positionMotor.currentPosition();
    positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
    positionMotor.setAcceleration(POSITION_ACCELERATION);
    positionMotor.moveTo(currentPos - (0.1 * POSITION_MOTOR_STEPS_PER_INCH));
    
    // Start cut motor returning home immediately
    cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
    cutMotor.moveTo(0);
    
    stage = 1; // Move to Step 1: Wait for 0.1 inch movement to complete
    return;
  }
  
  // Step 1: Wait for 0.1 inch movement to complete, then disengage position clamp and re-engage wood secure clamp
  if (stage == 1) {
    if (positionMotor.distanceToGo() == 0) {
      // After 0.1 inch, disengage position clamp
      digitalWrite(POSITION_CLAMP, HIGH);
      
      // Re-engage wood secure clamp
      digitalWrite(WOOD_SECURE_CLAMP, LOW);
      
      // Move directly to next stage without waiting
      stage = 2; // Move to Step 2: Return position motor to home position
    }
    return;
  }
  
  // Step 2: Return position motor to home position
  if (stage == 2) {
    // Set up position motor for returning home
    positionMotor.setMaxSpeed(POSITION_RETURN_SPEED);
    positionMotor.setAcceleration(POSITION_RETURN_ACCELERATION);
    positionMotor.moveTo(0);
    
    stage = 3; // Move to Step 3: Wait for position motor to reach home
    return;
  }
  
  // Step 3: Wait for position motor to reach home, engage position clamp
  if (stage == 3) {
    // First check if position motor has reached home
    if (positionMotor.distanceToGo() == 0) {
      // Position motor is now at home position
      // Engage position clamp immediately
      digitalWrite(POSITION_CLAMP, LOW);
      
      // Move to next stage to check cut motor
      stage = 4;
    }
    return;
  }
  
  // Step 4: Verify cut motor has properly returned home by checking homing sensor
  if (stage == 4) {
    // Wait for cut motor to complete its movement
    if (cutMotor.distanceToGo() == 0) {
      // Verify cut motor is properly homed by checking the homing sensor
      if (digitalRead(CUT_MOTOR_HOMING_SENSOR) == HIGH) { // HIGH means sensor is activated
        // Cut motor is properly homed, proceed to next step
        stage = 5; // Move to Step 5: Move position motor to final position
      } else {
        // Cut motor is not properly homed, enter error state
        currentState = ERROR;
        errorStartTime = millis();
        stage = 0; // Reset for next cycle
      }
    }
    return;
  }
  
  // Step 5: Move position motor to final position
  if (stage == 5) {
    // Move position motor to final position
    positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
    positionMotor.setAcceleration(POSITION_ACCELERATION);
    positionMotor.moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    
    stage = 6; // Move to Step 6: Wait for position motor to reach final position
    return;
  }
  
  // Step 6: Wait for position motor to reach final position
  if (stage == 6) {
    if (positionMotor.distanceToGo() == 0) {
      stage = 7; // Move to Step 7: Complete operation
    }
    return;
  }
  
  // Step 7: Complete operation
  if (stage == 7) {
    // Engage wood secure clamp
    digitalWrite(WOOD_SECURE_CLAMP, LOW);
    
    // Reset cycle flags
    cuttingCycleInProgress = false;
    signalSent = false;
    
    // Check for wood suction error
    if (woodSuctionError) {
      // Enter error state
      currentState = ERROR;
      errorStartTime = millis();
    } else {
      // Return to ready state
      currentState = READY;
      
      // Turn off busy LED
      digitalWrite(YELLOW_LED, LOW);
      
      // Turn on appropriate LED based on wood presence
      if (!woodPresent) {
        digitalWrite(BLUE_LED, HIGH); // No wood LED
      }
    }
    
    stage = 0; // Reset to beginning for next cycle
    return;
  }
}

void NOwood() {
  // Step sequence for no wood operation:
  // 0: Initialize - Set clamps and prepare for operation
  // 1: Move cut motor to home position and wait for it to complete
  // 2: Move position motor to home position
  // 3: Perform position motor cycle (out and back)
  // 4: Complete operation and return to ready state
  
  static int stage = 0;
  static unsigned long delayTimer = 0;
  
  // Step 0: Initialize - Set clamps and prepare for operation
  if (stage == 0) {
    // Set initial clamp states
    digitalWrite(POSITION_CLAMP, LOW);     // Engage position clamp
    digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Disengage wood secure clamp
    
    // Configure cut motor for return speed (faster than homing)
    cutMotor.setMaxSpeed(100);
    cutMotor.setAcceleration(100);
    
    // Immediately start moving cut motor to home position
    cutMotor.moveTo(0);
    while (cutMotor.distanceToGo() != 0) {
      cutMotor.run();
    }
    // Turn on yellow LED to indicate operation in progress
    digitalWrite(YELLOW_LED, HIGH);
    
    // Move to next stage
    stage = 1;
    return;
  }
  
  // Step 1: Move cut motor to home position and wait for it to complete
  if (stage == 1) {
    // Simply wait for cut motor to complete its movement to position 0
    if (cutMotor.distanceToGo() == 0) {
      // Proceed to next stage without checking home sensor
      stage = 2;
    }
    return;
  }
  
  // Step 2: Move position motor to home position
  if (stage == 2) {
    // Move position motor to home
    positionMotor.moveTo(0);
    
    // Check if position motor has reached home
    if (positionMotor.distanceToGo() == 0) {
      // Disengage position clamp
      digitalWrite(POSITION_CLAMP, HIGH);
      delayTimer = 0;
      stage = 3;
    }
    return;
  }
  
  // Step 3: Perform position motor cycle (out and back)
  if (stage == 3) {
    static int positionSubStage = 0;
    
    // Wait for clamp to fully disengage
    if (positionSubStage == 0) {
      if (Wait(100, &delayTimer)) {
        // Move position motor out 2 inches
        positionMotor.moveTo(2.0 * POSITION_MOTOR_STEPS_PER_INCH);
        positionSubStage = 1;
      }
    }
    // Wait for position motor to reach 2 inches
    else if (positionSubStage == 1) {
      if (positionMotor.distanceToGo() == 0) {
        // Engage position clamp
        digitalWrite(POSITION_CLAMP, LOW);
        delayTimer = 0;
        positionSubStage = 2;
      }
    }
    // Wait for clamp to fully engage
    else if (positionSubStage == 2) {
      if (Wait(100, &delayTimer)) {
        // Move position motor back to home
        positionMotor.moveTo(0);
        positionSubStage = 3;
      }
    }
    // Wait for position motor to reach home
    else if (positionSubStage == 3) {
      if (positionMotor.distanceToGo() == 0) {
        // Reset for next time
        positionSubStage = 0;
        stage = 4;
      }
    }
    return;
  }
  
  // Step 4: Complete operation and return to ready state
  if (stage == 4) {
    // Disengage both clamps
    digitalWrite(POSITION_CLAMP, HIGH);
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    
    // Wait for clamps to fully disengage
    if (Wait(100, &delayTimer)) {
      // Reset cycle flags
      cuttingCycleInProgress = false;
      
      // Return to ready state
      currentState = READY;
      
      // Update LEDs
      digitalWrite(YELLOW_LED, LOW);  // Turn off busy LED
      digitalWrite(BLUE_LED, HIGH);   // Turn on no-wood LED
      
      // Reset stage for next cycle
      stage = 0;
    }
    return;
  }
}

void handleErrorState() {
  // Blink red LED to indicate error
  if (millis() - lastErrorBlinkTime > ERROR_BLINK_INTERVAL) {
    errorBlinkState = !errorBlinkState;
    digitalWrite(RED_LED, errorBlinkState);
    lastErrorBlinkTime = millis();
  }
  
  // Turn off all other LEDs
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  
  // Wait for reload switch to be pressed to acknowledge error
  // This is handled in the main loop
}

void resetFromError() {
  // Turn off error LED
  digitalWrite(RED_LED, LOW);
  
  // Reset error flags
  woodSuctionError = false;
  errorAcknowledged = false;
  
  // Reset motors
  cutMotor.setCurrentPosition(0);
  positionMotor.setCurrentPosition(0);
  
  // Return to homing state
  currentState = HOMING;
  
  // Reset cycle flags
  cuttingCycleInProgress = false;
  signalSent = false;
}