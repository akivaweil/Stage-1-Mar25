#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

// ===== PIN DEFINITIONS =====

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 22       // Controls the pulse signal to the cut motor driver - moves the sliding table saw carriage
#define CUT_MOTOR_DIR_PIN 23         // Controls the direction of the cut motor (forward/backward movement)

#define POSITION_MOTOR_PULSE_PIN 32  // Controls the pulse signal to the position motor driver - feeds wood forward for next cut or retrieves the wood during the NOwood operation.
#define POSITION_MOTOR_DIR_PIN 33    // Controls the direction of the position motor (forward/backward movement)

// Switch and Sensor Pin Definitions (Left side inputs)
#define CUT_MOTOR_POSITION_SWITCH 25      // Limit switch that detects when cut motor is at home position
#define POSITION_MOTOR_POSITION_SWITCH 27 // Limit switch that detects when position motor is at home position
#define RELOAD_SWITCH 14                  // Manual switch to enter reload mode - disengages clamps for material loading
#define WOOD_SENSOR 35                    // Sensor that detects if wood is present (LOW when wood is detected)

// Switch and Sensor Pin Definitions (Right side)
#define START_CYCLE_SWITCH 18          // Switch that initiates the cutting cycle when activated. This can be left on for continuous operation. The system will cut the wood until no wood is detecteed.
#define WAS_WOOD_SUCTIONED_SENSOR 5    // Sensor that checks if wood was properly suctioned during cutting (error detection)

// Clamp Pin Definitions
#define POSITION_CLAMP 13           // Controls the pneumatic clamp that holds the positioning mechanism (LOW = engaged)
#define WOOD_SECURE_CLAMP 15        // Controls the pneumatic clamp that secures the wood piece (LOW = engaged)

// LED Pin Definitions
#define RED_LED 26    // Error indicator LED - blinks during error conditions
#define YELLOW_LED 21 // Operation in progress indicator - on during cutting or when in reload mode
#define GREEN_LED 4   // Ready indicator - on when system is ready to begin a cutting cycle
#define BLUE_LED 2    // Setup/special mode indicator - on during startup or when no wood present

// Signal Output
#define SIGNAL_TO_STAGE_1TO2 19    // Output signal to the next stage in the process (Stage 2) when cutting is complete

// ===== CONSTANTS =====
// System States
enum SystemState {
  STARTUP,
  HOMING,
  READY,
  CUTTING,
  YESWOOD,  // Renamed from POSITIONING
  NOWOOD,   // When the wood board is finished cutting and we need to grab and pull out the remaing wood.
  ERROR
};

// Motor Configuration
#define CUT_MOTOR_STEPS_PER_INCH 63.5  // 200 steps/rev with 40T pulley on 2GT belt (80mm/rev)
#define POSITION_MOTOR_STEPS_PER_INCH 1000
#define CUT_TRAVEL_DISTANCE 8.5 // inches
#define POSITION_TRAVEL_DISTANCE 3.45 // inches
#define CUT_HOMING_DIRECTION -1
#define POSITION_HOMING_DIRECTION -1

// Speed and Acceleration Settings
#define CUT_NORMAL_SPEED 90
#define CUT_RETURN_SPEED 2000
#define CUT_ACCELERATION 2200
#define CUT_HOMING_SPEED 300
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
Bounce woodSensor = Bounce();
Bounce woodSuctionSensor = Bounce();

// System Flags
bool isHomed = false;
bool isReloadMode = false;
bool woodPresent = false;
bool woodSuctionError = false;
bool homingError = false;  // Flag to track homing errors
bool signalSent = false;
bool errorAcknowledged = false;
bool cuttingCycleInProgress = false;
bool continuousModeActive = false;
bool startSwitchSafe = false;
bool checkedForWoodSuction = false;
bool noWoodCycleCompleted = false;  // Flag to track if we just completed a NOwood operation

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
  
  // Check if start switch is already ON at startup
  startCycleSwitch.update();
  if (startCycleSwitch.read() == HIGH) {
    startSwitchSafe = false;
    // Serial.println("WARNING: Start switch is ON at startup");
  } else {
    startSwitchSafe = true;
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
  pinMode(CUT_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
  pinMode(POSITION_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
  pinMode(RELOAD_SWITCH, INPUT);                       // External pull-down
  pinMode(START_CYCLE_SWITCH, INPUT);                  // External pull-down
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
  digitalWrite(POSITION_CLAMP, LOW);      // Engage position clamp
  digitalWrite(WOOD_SECURE_CLAMP, LOW);   // Engage wood secure clamp
  digitalWrite(RED_LED, LOW);             // Turn off error LED
  digitalWrite(YELLOW_LED, LOW);          // Turn off busy LED
  digitalWrite(GREEN_LED, LOW);           // Turn off ready LED
  digitalWrite(BLUE_LED, HIGH);           // Turn on setup LED during startup
  digitalWrite(SIGNAL_TO_STAGE_1TO2, LOW);    // No signal to TA
  
  // Configure motors
  cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_ACCELERATION);
  positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_ACCELERATION);
  
  // Initialize bounce objects for debouncing switches
  cutHomingSensor.attach(CUT_MOTOR_POSITION_SWITCH);
  cutHomingSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  positionHomingSensor.attach(POSITION_MOTOR_POSITION_SWITCH);
  positionHomingSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  reloadSwitch.attach(RELOAD_SWITCH);
  reloadSwitch.interval(SIGNAL_DEBOUNCE_INTERVAL);
  startCycleSwitch.attach(START_CYCLE_SWITCH);
  startCycleSwitch.interval(SIGNAL_DEBOUNCE_INTERVAL);
  woodSensor.attach(WOOD_SENSOR);
  woodSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  woodSuctionSensor.attach(WAS_WOOD_SUCTIONED_SENSOR);
  woodSuctionSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  
  // Test and report homing sensor states
  // Serial.println("Initial sensor states:");
  // Serial.print("Cut homing sensor: ");
  // Serial.println(digitalRead(CUT_MOTOR_POSITION_SWITCH) == HIGH ? "HIGH (active)" : "LOW (inactive)");
  // Serial.print("Position homing sensor: ");
  // Serial.println(digitalRead(POSITION_MOTOR_POSITION_SWITCH) == HIGH ? "HIGH (active)" : "LOW (inactive)");
  
  // Set system as not homed
  isHomed = false;
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
  
  // Reset noWoodCycleCompleted flag when start switch is released
  if (noWoodCycleCompleted && startCycleSwitch.fell()) {
    noWoodCycleCompleted = false;
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
      digitalWrite(YELLOW_LED, HIGH);     // Turn on blue and yellow LED for reload mode
      digitalWrite(BLUE_LED, HIGH);
    } else if (!reloadSwitchOn && isReloadMode) {
      // Exit reload mode
      isReloadMode = false;
      digitalWrite(POSITION_CLAMP, LOW);   // Re-engage position clamp
      digitalWrite(WOOD_SECURE_CLAMP, LOW); // Re-engage wood secure clamp
      digitalWrite(YELLOW_LED, LOW);       // Turn off blue and yellow LED
      digitalWrite(BLUE_LED, LOW);
    }
  }
  
  // Handle error acknowledgment separately
  if (reloadSwitch.rose() && currentState == ERROR) {
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
      Ready();
      break;
      
    case CUTTING:
        // Set cut motor parameters and start movement
    cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
      CUTmovement();
      break;
      
    case YESWOOD:
      YESwood();
      break;
      
    case NOWOOD:
      NOwood();
      break;
      
    case ERROR:
      if (woodSuctionError) {
        suctionError();
      }
      if (homingError) {
        handleCutMotorHomingError();
      }
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
  static unsigned long blinkTimer = 0;
  
  // Blink blue LED to indicate homing
  if (millis() - blinkTimer > BLINK_INTERVAL) {
    blinkState = !blinkState;
    digitalWrite(BLUE_LED, blinkState);
    blinkTimer = millis();
  }
  
  // Home cut motor first
  if (!cutMotorHomed) {
    // If the cut motor homing sensor is already active, move away first
    if (cutHomingSensor.read() == HIGH) {
      // Use full speed when moving away from the sensor
      cutMotor.setMaxSpeed(CUT_HOMING_SPEED);
      cutMotor.moveTo(10 * CUT_MOTOR_STEPS_PER_INCH); // Move slightly away from sensor
      if (cutMotor.distanceToGo() == 0) {
        // Now move back to find the sensor using the homing speed directly
        cutMotor.setSpeed(CUT_HOMING_SPEED * CUT_HOMING_DIRECTION);
        cutMotor.moveTo(-10000); // Move toward sensor (will stop when sensor activated)
      }
    } else {
      // Move toward home sensor using the homing speed directly
      cutMotor.setSpeed(CUT_HOMING_SPEED * CUT_HOMING_DIRECTION);
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
      // Move toward home sensor using the homing speed directly
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
    
    // Set system as homed
    isHomed = true;
    
    // Turn off blue LED, turn on green LED
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    
    // Transition to READY state
    currentState = READY;
  }
}

void Ready() {
      // System is ready for operation
      if (!isReloadMode) {
        // Solid green LED to indicate ready
        digitalWrite(GREEN_LED, HIGH);
        
        // Start a new cycle if:
        // 1. Start switch was just flipped ON (rising edge), OR
        // 2. Continuous mode is active AND we're not already in a cutting cycle
        // AND the start switch is safe to use
        // AND no errors are active
        // AND we haven't just completed a NOwood cycle (or if we have, the start switch was released and re-engaged)
        if (((continuousModeActive && !cuttingCycleInProgress) 
            && !woodSuctionError && !homingError) && startSwitchSafe && !noWoodCycleCompleted) {
          // Turn off ready LED, turn on busy LED
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(YELLOW_LED, HIGH);
          digitalWrite(BLUE_LED, LOW);  // Ensure blue LED is off at start of cutting cycle
          
          // Set flag to indicate cycle in progress
          cuttingCycleInProgress = true;
          
          // Always enter cutting state, regardless of wood presence
          currentState = CUTTING;
          
          // Store wood presence for later use
          if (!woodPresent) {
            digitalWrite(BLUE_LED, HIGH); // Blue LED in preparation for no-wood mode
          }
        }
      }
}

void CUTmovement() {
  // Cutting operation:
  // 1. Ensure both clamps are engaged
  // 2. Move cut motor to cutting position
  // 3. Signal TA when complete (active HIGH)
  // 4. Transition to YESwood or NOwood based on wood presence
  

  static unsigned long clampTimer = 0;

  
  // Step 1: Ensure both clamps are engaged before starting cut movement {
    // Engage both clamps
    digitalWrite(POSITION_CLAMP, LOW);      // Engage position clamp
    digitalWrite(WOOD_SECURE_CLAMP, LOW);   // Engage wood secure clamp
  
  // Step 1: Start the cut motor movement
    // Set cut motor parameters and start movement
    cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
    
    // Move to cutting position
    long targetPosition = CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH;
    cutMotor.moveTo(targetPosition);
    
  
  // Step 2: Wait for cutting motion to complete and check for wood suction error
    // Reset the wood suction check flag
    checkedForWoodSuction = false;
    // Check for wood suction error when the cut motor is 0.5 inches into the cut
    if (cutMotor.currentPosition() >= (0.3 * CUT_MOTOR_STEPS_PER_INCH)) {
      // Check if wood was suctioned (active LOW)
      if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) {
        // Set the wood suction error flag
        woodSuctionError = true;
        
        // Stop the cut motor
        cutMotor.stop();
        
        // Enter error state
        currentState = ERROR;
        errorStartTime = millis();
      
      // Mark that we've checked for wood suction
      checkedForWoodSuction = true;
    }
    
    if (cutMotor.distanceToGo() == 0) {
      // Re-read the wood sensor after cut move finishes
      woodPresent = (digitalRead(WOOD_SENSOR) == LOW); // LOW indicates wood present
      
      // Send signal to TA (Transfer Arm) regardless of wood presence
      digitalWrite(SIGNAL_TO_STAGE_1TO2, HIGH);
      signalSent = true;
      
      // Reset stage for next cycle
      
      // Wait a moment for the signal to be received
      static unsigned long signalTimer = 0;
      if (Wait(200, &signalTimer)) {
        // Reset signal (LOW)
        digitalWrite(SIGNAL_TO_STAGE_1TO2, LOW);
        
        if (woodPresent) {
          // Wood present, move to YESwood state
          currentState = YESWOOD;
        } else {
          // No wood present, change to NOwood state
          currentState = NOWOOD;  // Use the new NOWOOD state
        }
      }
    }
    return;
}
}
void YESwood() {
  static int stage = 0;
  static long currentPos;
  static unsigned long timer = 0;
  static bool inSlowApproachMode = false;  // Flag to track if we're in slow approach mode
  
  // Always run both motors to ensure they continue moving
  positionMotor.run();
  cutMotor.run();
  
  // Stage 0: Initialization - start both motors moving toward home
  if (stage == 0) {
    // Engage position clamp during initial movement
    digitalWrite(POSITION_CLAMP, LOW);
    
    // Retract wood secure clamp
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    
    // Start position motor moving to home position
    positionMotor.setMaxSpeed(POSITION_RETURN_SPEED);
    positionMotor.setAcceleration(POSITION_RETURN_ACCELERATION);
    positionMotor.moveTo(0);  // Move directly to home position
    
    // Initiate cut motor fast return to home - set a target well past home
    // This ensures we won't reach the target before hitting the sensor
    cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
    cutMotor.moveTo(-2 * CUT_MOTOR_STEPS_PER_INCH);  // Target beyond home position
    
    // Reset the slow approach mode flag
    inSlowApproachMode = false;
    
    stage = 1;
    return;
  }
  
  // Stage 1: Dynamic speed control for cut motor approaching home sensor
  if (stage == 1) {
    // Release position clamp after a short delay to allow initial movement
    static unsigned long clampTimer = 0;
    if (clampTimer == 0) {
      clampTimer = millis();
    }
    
    if (millis() - clampTimer > 100) {
      digitalWrite(POSITION_CLAMP, HIGH);  // Release position clamp
      clampTimer = 0;  // Reset timer
    }
    
    // Dynamically adjust speed based on position - WITHOUT changing the target
    if (!inSlowApproachMode) {
      // Calculate approximate distance to home based on current position
      long estimatedDistanceToHome = abs(cutMotor.currentPosition());
      
      // When we get within 0.5 inches of home position, reduce speed
      if (estimatedDistanceToHome < (0.5 * CUT_MOTOR_STEPS_PER_INCH)) {
        cutMotor.setMaxSpeed(CUT_RETURN_SPEED / 5);  // 20% of return speed
        inSlowApproachMode = true;
        // The motor continues to the SAME target, just slower now
      }
    }
    
    // Check if we've hit the homing sensor
    cutHomingSensor.update();
    if (cutHomingSensor.rose() || cutHomingSensor.read() == HIGH) {
      // Stop the motor when sensor is triggered
      cutMotor.stop();
      cutMotor.setCurrentPosition(0);  // Set current position as home
      
      // Check if position motor has reached home
      if (positionMotor.distanceToGo() == 0) {
        // Both motors are home, proceed to next stage
        stage = 3;  // Skip to stage 3 since we're already at home
        inSlowApproachMode = false;  // Reset the flag
        return;
      }
      // Otherwise, continue waiting for position motor in stage 2
      stage = 2;
      inSlowApproachMode = false;  // Reset the flag
    }
    return;
  }
  
  // Stage 2: Wait for position motor to reach home
  if (stage == 2) {
    // Check if position motor has reached home
    if (positionMotor.distanceToGo() == 0) {
      // Both motors are home, proceed to next stage
      stage = 3;
    }
    return;
  }
  
  // Stage 3: Engage position clamp now that both motors are at home
  if (stage == 3) {
    digitalWrite(POSITION_CLAMP, LOW);  // Engage position clamp
    stage = 4;
    return;
  }
  
  // Stage 4: Move position motor to final position
  if (stage == 4) {
    positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
    positionMotor.setAcceleration(POSITION_ACCELERATION);
    // Ensure wood secure clamp is retracted during movement
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    positionMotor.moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
    stage = 5;
    return;
  }
  
  // Stage 5: Wait for position motor to reach final position and retract wood secure clamp
  if (stage == 5) {
    if (positionMotor.distanceToGo() == 0) {
      // Engage wood secure clamp after position motor reaches final position
      digitalWrite(WOOD_SECURE_CLAMP, LOW);
      stage = 6;
    }
    return;
  }
  
  // Stage 6: Complete operation
  if (stage == 6) {
    cuttingCycleInProgress = false;
    signalSent = false;
    if (woodSuctionError || homingError) {
      currentState = ERROR;
      errorStartTime = millis();
    } else {
      currentState = READY;
      digitalWrite(YELLOW_LED, LOW);
      if (!woodPresent) {
        digitalWrite(BLUE_LED, HIGH);
      }
    }
    stage = 0;
    return;
  }
}

void NOwood() {
  // Function that moves both motors back to home position simultaneously
  static int stage = 0;
  static unsigned long waitTimer = 0;
  
  // Always run the motors to ensure they continue moving
  cutMotor.run();
  positionMotor.run();
  
  // Step 0: Initialize - Configure both motors for return
  if (stage == 0) {
    // Configure cut motor for return speed
    cutMotor.setMaxSpeed(CUT_RETURN_SPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
    
    // Configure position motor for return
    positionMotor.setMaxSpeed(POSITION_RETURN_SPEED);
    positionMotor.setAcceleration(POSITION_RETURN_ACCELERATION);
    
    // Start moving both motors to home position
    cutMotor.moveTo(0);
    positionMotor.moveTo(0);
    
    // Retract the secure wood clamp
    digitalWrite(WOOD_SECURE_CLAMP, HIGH);
    
    // Turn on yellow LED to indicate operation in progress
    digitalWrite(YELLOW_LED, HIGH);
    
    // Move to next stage
    stage = 1;
    return;
  }
  
  // Step 1: Wait for both motors to complete their movement to home position
  if (stage == 1) {
    // Check if both motors have completed their movements
    if (cutMotor.distanceToGo() == 0 && positionMotor.distanceToGo() == 0) {
      // Both motors are at home, release the position clamp
      digitalWrite(POSITION_CLAMP, HIGH);
      
      // Add a small delay to ensure clamp is fully released
      waitTimer = 0;
      stage = 2;
    }
    return;
  }
  
  // Step 2: Wait for a short delay after releasing clamp
  if (stage == 2) {
    if (Wait(500, &waitTimer)) {
      // Reset cycle flags
      cuttingCycleInProgress = false;
      signalSent = false;
      
      // Reset global flags
      checkedForWoodSuction = false;
      
      // Set flag to indicate NOwood cycle just completed
      noWoodCycleCompleted = true;
      
      // Return to ready state
      currentState = READY;
      
      // Update LEDs
      digitalWrite(YELLOW_LED, LOW);  // Turn off busy LED
      digitalWrite(BLUE_LED, HIGH);   // Turn on no-wood LED
      
      // Reset stage for next cycle
      stage = 0;
      waitTimer = 0;
    }
    return;
  }
}

void handleCutMotorHomingError() {
  // Blink red and yellow LEDs to indicate error
  if (millis() - lastErrorBlinkTime > ERROR_BLINK_INTERVAL) {
    errorBlinkState = !errorBlinkState;
    digitalWrite(RED_LED, errorBlinkState);
    digitalWrite(YELLOW_LED, errorBlinkState);
    lastErrorBlinkTime = millis();
  }
  
  // Turn off all other LEDs
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  
  // Check if error was acknowledged with reload switch
  if (errorAcknowledged) {
    // Turn off error LEDs
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    
    // Reset error flags
    homingError = false;
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
}

void suctionError() {
  // Blink red and yellow LEDs to indicate error
  if (millis() - lastErrorBlinkTime > ERROR_BLINK_INTERVAL) {
    errorBlinkState = !errorBlinkState;
    digitalWrite(RED_LED, errorBlinkState);
    digitalWrite(YELLOW_LED, errorBlinkState);
    lastErrorBlinkTime = millis();
  }
  
  // Turn off all other LEDs
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  
  // Check if error was acknowledged with reload switch
  if (errorAcknowledged) {
    // Turn off error LEDs
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    
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
}