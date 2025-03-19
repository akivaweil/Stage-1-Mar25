#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

// ===== PIN DEFINITIONS =====

// Motor Pin Definitions
#define CUT_MOTOR_PULSE_PIN 48       // Controls the pulse signal to the cut motor driver - moves the sliding table saw carriage
#define CUT_MOTOR_DIR_PIN 47         // Controls the direction of the cut motor (forward/backward movement)

#define POSITION_MOTOR_PULSE_PIN 21  // Controls the pulse signal to the position motor driver - feeds wood forward for next cut or retrieves the wood during the NOwood operation.
#define POSITION_MOTOR_DIR_PIN 20    // Controls the direction of the position motor (forward/backward movement)

// Switch and Sensor Pin Definitions (Left side inputs)
#define CUT_MOTOR_POSITION_SWITCH 10      // Limit switch that detects when cut motor is at home position
#define POSITION_MOTOR_POSITION_SWITCH 9 // Limit switch that detects when position motor is at home position
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
#define CUTTINGSPEED 80
#define CUT_RETURN_SPEED 2000
#define CUT_ACCELERATION 2200
#define CUT_HOMING_SPEED 300
#define POSITION_NORMAL_SPEED 30000
#define POSITION_RETURN_SPEED 30000
#define POSITION_ACCELERATION 30000
#define POSITION_RETURN_ACCELERATION 30000
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
Bounce cycleSwitch = Bounce();
Bounce woodSensor = Bounce();
Bounce woodSuctionSensor = Bounce();

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
bool skipPositionHoming = false;      // Flag to skip position motor homing when true

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
  pinMode(CUT_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
  pinMode(POSITION_MOTOR_POSITION_SWITCH, INPUT_PULLUP);
  pinMode(RELOAD_SWITCH, INPUT);                       // External pull-down
  pinMode(CYCLE_SWITCH, INPUT);                  // External pull-down
  pinMode(WOOD_SENSOR, INPUT_PULLUP);                  // Active LOW (LOW = wood present)
  pinMode(WAS_WOOD_SUCTIONED_SENSOR, INPUT_PULLUP);    // Active LOW (LOW = error)
  
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
  cutHomingSensor.attach(CUT_MOTOR_POSITION_SWITCH);
  cutHomingSensor.interval(SIGNAL_DEBOUNCE_INTERVAL);
  positionHomingSensor.attach(POSITION_MOTOR_POSITION_SWITCH);
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
  static bool cutMotorMovedAway = false;  // New flag to track completion of move-away
  static bool positionMotorMovedAway = false;  // New flag to track completion of move-away
  static unsigned long blinkTimer = 0;
  
  // Blink blue LED to indicate homing in progress
  if (millis() - blinkTimer > BLINK_INTERVAL) {
    blinkState = !blinkState;
    digitalWrite(BLUE_LED, blinkState);
    blinkTimer = millis();
  }
  
  // Step 1: Home the cut motor first
  if (!cutMotorHomed) {
    // Check if sensor is already triggered (motor at home position)
    if (cutHomingSensor.read() == HIGH && !cutMotorMovedAway) {
      // Move exactly 1 inch away from sensor first to ensure proper homing
      cutMotor.setMaxSpeed(CUT_HOMING_SPEED);
      cutMotor.setAcceleration(CUT_ACCELERATION / 8);
      cutMotor.moveTo(2 * CUT_MOTOR_STEPS_PER_INCH); // Move 1 inch away from sensor
      
      // Check if the 1-inch move-away is complete
      if (cutMotor.distanceToGo() == 0) {
        cutMotorMovedAway = true;  // Mark move-away as complete
      }
    } 
    // After 1-inch move-away is complete or if sensor wasn't triggered initially
    else {
      // If sensor wasn't triggered initially, no need to move away
      if (!cutHomingSensor.read() == HIGH) {
        cutMotorMovedAway = true;
      }
      
      // Only start moving toward home after move-away is complete
      if (cutMotorMovedAway) {
        // Now move toward sensor to find home
        // KEEP THE SAME ACCELERATION as the move-away operation to prevent jerky movement
        cutMotor.setMaxSpeed(CUT_HOMING_SPEED);
        cutMotor.setAcceleration(CUT_ACCELERATION / 8);
        cutMotor.setSpeed(CUT_HOMING_SPEED * CUT_HOMING_DIRECTION);
        cutMotor.moveTo(-10000); // Large negative value to ensure movement
      }
      
      // Check if home sensor has been triggered
      if (cutHomingSensor.read() == HIGH) {
        cutMotor.stop();
        cutMotor.setCurrentPosition(0); // Set current position as zero
        cutMotorHomed = true;
        cutMotorMovedAway = false;  // Reset for next time
      }
    }
  } 
  // Step 2: Home the position motor after cut motor (skip if skipPositionHoming is true)
  else if (!positionMotorHomed) {
    if (skipPositionHoming) {
      // If we're skipping position motor homing, mark it as already homed
      positionMotorHomed = true;
      
      // Don't reset position motor's current position
      // But ensure the clamp is in the right state
      digitalWrite(POSITION_CLAMP, HIGH);  // Retract position clamp
    } else {
      // Retract position clamp during homing
      digitalWrite(POSITION_CLAMP, HIGH);
      
      // Check if sensor is already triggered
      if (positionHomingSensor.read() == HIGH && !positionMotorMovedAway) {
        // Move exactly 1 inch away from sensor first
        positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED);
        positionMotor.moveTo(1 * POSITION_MOTOR_STEPS_PER_INCH); // Move 1 inch away
        
        // Check if the 1-inch move-away is complete
        if (positionMotor.distanceToGo() == 0) {
          positionMotorMovedAway = true;  // Mark move-away as complete
        }
      } 
      // After 1-inch move-away is complete or if sensor wasn't triggered initially
      else {
        // If sensor wasn't triggered initially, no need to move away
        if (!positionHomingSensor.read() == HIGH) {
          positionMotorMovedAway = true;
        }
        
        // Only start moving toward home after move-away is complete
        if (positionMotorMovedAway) {
          // Now move toward sensor
          positionMotor.setSpeed(POSITION_HOMING_SPEED * POSITION_HOMING_DIRECTION);
          positionMotor.moveTo(-10000 * POSITION_MOTOR_STEPS_PER_INCH);
        }
        
        // Check if home sensor has been triggered
        if (positionHomingSensor.read() == HIGH) {
          positionMotor.stop();
          positionMotor.setCurrentPosition(-1 * POSITION_MOTOR_STEPS_PER_INCH); // Set -1 inch position
          positionMotorHomed = true;
          positionMotorMovedAway = false;  // Reset for next time
          positionMotor.setMaxSpeed(POSITION_NORMAL_SPEED); // Reset speed to normal
        }
      }
    }
  } 
  // Step 3: Move position motor to operating position (skip if skipPositionHoming is true)
  else if (!positionMotorMoved) {
    if (skipPositionHoming) {
      // If we're skipping position motor homing, mark it as already moved to position
      positionMotorMoved = true;
      
      // Ensure position clamp is extended
      digitalWrite(POSITION_CLAMP, LOW); // Extend position clamp
    } else {
      // Move position motor to final position
      positionMotor.moveTo(3.45 * POSITION_MOTOR_STEPS_PER_INCH);
      
      if (positionMotor.distanceToGo() == 0) {
        positionMotorMoved = true;
        digitalWrite(POSITION_CLAMP, LOW); // Extend position clamp
      }
    }
  } 
  // Step 4: Complete homing sequence
  else {
    // Reset homing flags for next time
    cutMotorHomed = false;
    positionMotorHomed = false;
    positionMotorMoved = false;
    
    // Reset the skip position homing flag
    skipPositionHoming = false;
    
    // Set system as homed and transition to READY state
    isHomed = true;
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    currentState = READY;
  }
}

void Ready() {
      // System is ready for operation
      if (!isReloadMode) {
        // Solid green LED to indicate ready
        digitalWrite(GREEN_LED, HIGH);
        
        // Start a new cycle if:
        // 1. Cycle switch was just flipped ON (rising edge), OR
        // 2. Continuous mode is active AND we're not already in a cutting cycle
        // AND the cycle switch has been reset if needed
        // AND no errors are active
        // AND we haven't just completed a NOwood cycle (or if we have, the cycle switch was released and re-engaged)
        if (((isContinuousModeActive && !isCuttingCycleInProgress) 
            && !hasWoodSuctionError && !hasHomingError) && !requireCycleSwitchReset && !isNoWoodCycleCompleted) {
          // Turn off ready LED, turn on busy LED
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(YELLOW_LED, HIGH);
          digitalWrite(BLUE_LED, LOW);  // Ensure blue LED is off at start of cutting cycle
          
          // Set flag to indicate cycle in progress
          isCuttingCycleInProgress = true;
          
          // Always enter cutting state, regardless of wood presence
          currentState = CUTTING;
          
          // Store wood presence for later use
          if (!isWoodPresent) {
            digitalWrite(BLUE_LED, HIGH); // Blue LED in preparation for no-wood mode
          }
        }
      }
}

void CUTmovement() {
  // Cutting operation:
  // 1. Ensure both clamps are extended
  // 2. Move cut motor to cutting position
  // 3. Signal TA when complete (active HIGH)
  // 4. Transition to YESwood or NOwood based on wood presence
  
  static unsigned long clampTimer = 0;
  static bool cycleStarted = false;
  
  // Reset flags at the beginning of each cut cycle
  if (!cycleStarted) {
    hasCheckedForWoodSuction = false;
    cycleStarted = true;
  }
  
  // Set cut motor parameters and start movement
  cutMotor.setMaxSpeed(CUTTINGSPEED);
  cutMotor.setAcceleration(CUT_ACCELERATION);
  
  // Step 1: Ensure both clamps are extended before starting cut movement {
    // Extend both clamps
    digitalWrite(POSITION_CLAMP, LOW);      // Extend position clamp
    digitalWrite(WOOD_SECURE_CLAMP, LOW);   // Extend wood secure clamp
  
  // Step 1: Start the cut motor movement
    // Set cut motor parameters and start movement
    cutMotor.setMaxSpeed(CUTTINGSPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
    
    // Move to cutting position
    long targetPosition = CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH;
    cutMotor.moveTo(targetPosition);
    
  
  // Step 2: Wait for cutting motion to complete and continuously check for wood suction error
    if (cutMotor.currentPosition() >= (0.3 * CUT_MOTOR_STEPS_PER_INCH) && 
        cutMotor.currentPosition() <= (0.5 * CUT_MOTOR_STEPS_PER_INCH) && 
        !hasCheckedForWoodSuction) {
      // Now we're at the position to check - read the sensor directly
      // For WAS_WOOD_SUCTIONED_SENSOR, LOW means error
      woodSuctionSensor.update();
      int sensorValue = digitalRead(WAS_WOOD_SUCTIONED_SENSOR);

      // If wood is present and sensor is LOW, trigger error
      if (isWoodPresent && sensorValue == LOW) {
          hasWoodSuctionError = true;
          cutMotor.stop();
          currentState = ERROR;
          errorStartTime = millis();
          return;
      }
      
      // Mark that we've checked for wood suction
      hasCheckedForWoodSuction = true;
    }
    
    if (cutMotor.distanceToGo() == 0) {
      // Re-read the wood sensor after cut move finishes
      isWoodPresent = (digitalRead(WOOD_SENSOR) == LOW); // LOW indicates wood present
      
      // Send signal to TA (Transfer Arm) regardless of wood presence
      digitalWrite(SIGNAL_TO_STAGE_1TO2, HIGH);
      isSignalSent = true;
      
      // Reset cycle flags
      cycleStarted = false;
      
      // Wait a moment for the signal to be received
      static unsigned long signalTimer = 0;
      if (Wait(200, &signalTimer)) {
        // Reset signal (LOW)
        digitalWrite(SIGNAL_TO_STAGE_1TO2, LOW);
        
        if (isWoodPresent) {
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
    // Extend position clamp during initial movement
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
    // Retract position clamp after a short delay to allow initial movement
    static unsigned long clampTimer = 0;
    if (clampTimer == 0) {
      clampTimer = millis();
    }
    
    if (millis() - clampTimer > 100) {
      digitalWrite(POSITION_CLAMP, HIGH);  // Retract position clamp
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
  
  // Stage 3: Extend position clamp now that both motors are at home
  if (stage == 3) {
    digitalWrite(POSITION_CLAMP, LOW);  // Extend position clamp
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
  
  // Stage 5: Wait for position motor to reach final position and extend wood secure clamp
  if (stage == 5) {
    if (positionMotor.distanceToGo() == 0) {
      // Extend wood secure clamp after position motor reaches final position
      digitalWrite(WOOD_SECURE_CLAMP, LOW);
      stage = 6;
    }
    return;
  }
  
  // Stage 6: Complete operation
  if (stage == 6) {
    isCuttingCycleInProgress = false;
    isSignalSent = false;
    if (hasWoodSuctionError || hasHomingError) {
      currentState = ERROR;
      errorStartTime = millis();
    } else {
      currentState = READY;
      digitalWrite(YELLOW_LED, LOW);
      if (!isWoodPresent) {
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
  static bool cutMotorRehomed = false;
  
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
    
    // Reset the rehomed flag
    cutMotorRehomed = false;
    
    // Move to next stage
    stage = 1;
    return;
  }
  
  // Step 1: Wait for both motors to complete their movement to home position
  if (stage == 1) {
    // Check if both motors have completed their movements
    if (cutMotor.distanceToGo() == 0 && positionMotor.distanceToGo() == 0) {
      // Both motors are at home, retract the position clamp
      digitalWrite(POSITION_CLAMP, HIGH);
      
      // Add a small delay to ensure clamp is fully retracted
      waitTimer = 0;
      stage = 2;
    }
    return;
  }
  
  // Step 2: Wait for a short delay after retracting clamp
  if (stage == 2) {
    if (Wait(500, &waitTimer)) {
      // Move cut motor 0.5 inches away from home
      cutMotor.setMaxSpeed(CUT_HOMING_SPEED);
      cutMotor.moveTo(0.5 * CUT_MOTOR_STEPS_PER_INCH);
      
      // Move to next stage
      stage = 3;
      waitTimer = 0;
    }
    return;
  }
  
  // Step 3: Wait for cut motor to move 0.5 inches away from home
  if (stage == 3) {
    if (cutMotor.distanceToGo() == 0) {
      // Now rehome the cut motor
      // First set parameters for homing
      cutMotor.setMaxSpeed(CUT_HOMING_SPEED / 5); // 5x slower for precise homing
      cutMotor.setAcceleration(CUT_ACCELERATION / 10);
      
      // Move toward home sensor using continuous motion in the homing direction at 5x slower speed
      cutMotor.setSpeed((CUT_HOMING_SPEED / 5) * CUT_HOMING_DIRECTION);
      
      // Move to next stage
      stage = 4;
    }
    return;
  }
  
  // Step 4: Wait for cut motor to rehome
  if (stage == 4) {
    // Check if home sensor has been triggered
    cutHomingSensor.update();
    
    // Use setSpeed for continuous motion until sensor is triggered
    if (cutHomingSensor.read() != HIGH) {
      // Continue moving at constant speed in homing direction until sensor is hit
      // Use 5x slower speed for more precise homing
      cutMotor.setSpeed((CUT_HOMING_SPEED / 5) * CUT_HOMING_DIRECTION);
      cutMotor.runSpeed(); // Direct control of motor at constant speed
    } else {
      // Stop the motor when sensor is triggered
      cutMotor.stop();
      cutMotor.setCurrentPosition(0); // Set current position as home
      
      // Complete the cycle
      stage = 5;
    }
    return;
  }
  
  // Step 5: Complete the cycle
  if (stage == 5) {
    // Reset cycle flags
    isCuttingCycleInProgress = false;
    isSignalSent = false;
    
    // Reset global flags
    hasCheckedForWoodSuction = false;  // Make sure this is reset for the next cycle
    
    // Set flag to indicate NOwood cycle just completed
    isNoWoodCycleCompleted = true;
    
    // Return to ready state
    currentState = READY;
    
    // Update LEDs
    digitalWrite(YELLOW_LED, LOW);  // Turn off busy LED
    digitalWrite(BLUE_LED, HIGH);   // Turn on no-wood LED
    
    // Reset stage for next cycle
    stage = 0;
    waitTimer = 0;
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
  if (isErrorAcknowledged) {
    // Turn off error LEDs
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    
    // Reset error flags
    hasHomingError = false;
    isErrorAcknowledged = false;
    
    // Return to homing state
    currentState = HOMING;
    
    // Reset cycle flags
    isCuttingCycleInProgress = false;
    isSignalSent = false;
  }
}

void suctionError() {
  // Blink blue and yellow LEDs to indicate error
  if (millis() - lastErrorBlinkTime > ERROR_BLINK_INTERVAL) {
    errorBlinkState = !errorBlinkState;
    digitalWrite(BLUE_LED, errorBlinkState);
    digitalWrite(YELLOW_LED, errorBlinkState);
    lastErrorBlinkTime = millis();
  }
  
  // Turn off all other LEDs
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  
  // Auto-reset after 2 seconds (2000ms)
  if (millis() - errorStartTime > 2000) {
    // Turn off error LEDs
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    
    // Reset error flags
    hasWoodSuctionError = false;
    
    // Require cycle switch to be reset before allowing new cycle
    requireCycleSwitchReset = true;
    
    // Skip position motor homing
    skipPositionHoming = true;
    
    // Return to homing state
    currentState = HOMING;
    
    // Reset cycle flags
    isCuttingCycleInProgress = false;
    isSignalSent = false;
  }
  
  // Check if error was acknowledged with reload switch
  if (isErrorAcknowledged) {
    // Turn off error LEDs
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    
    // Reset error flags
    hasWoodSuctionError = false;
    isErrorAcknowledged = false;
    
    // Require cycle switch to be reset before allowing new cycle
    requireCycleSwitchReset = true;
    
    // Skip position motor homing
    skipPositionHoming = true;
    
    // Return to homing state
    currentState = HOMING;
    
    // Reset cycle flags
    isCuttingCycleInProgress = false;
    isSignalSent = false;
  }
}