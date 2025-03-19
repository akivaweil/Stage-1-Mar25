# Wood Cutting Machine - Stage 1 Functional Notes

## Overview
This automated wood cutting machine uses two stepper motors and two pneumatic clamps to precisely cut wood boards to specified lengths. The system follows a specific cutting cycle sequence with integrated safety features, homing capabilities, and error detection systems.

## Hardware Components

### Motors
1. **Cut Motor** - Controls the cutting blade movement across the 8.5-inch travel path
2. **Position Motor** - Controls the wood positioning for precise movement across a 3.45-inch travel path

### Clamps
1. **Position Clamp** - Secures the wood during positioning operations and provides additional support during cutting
2. **Secure Wood Clamp** - Firmly holds the wood in position during cutting operations

### Switches and Sensors
1. **Cut Motor Position Switch** - Limit switch that detects when the cut motor is at home position (HIGH when triggered)
2. **Position Motor Position Switch** - Limit switch that detects when the position motor is at home position (HIGH when triggered)
3. **Reload Switch** - Activates reload mode for loading new wood (HIGH when triggered)
4. **Cycle Switch** - Initiates the cutting cycle (HIGH when triggered)
5. **YESorNO_WOOD_SENSOR** - Detects the presence of wood (LOW when wood is present)
6. **WAS_WOOD_SUCTIONED_SENSOR** - Verifies proper wood suction during cutting operations (HIGH when suction is correct, LOW indicates failure/error)

### LED Indicators
1. **Red LED** - Error indicator - blinks with pattern indicating specific error type
2. **Yellow LED** - Operation in progress indicator
3. **Green LED** - Ready indicator - illuminates when system is ready for cutting cycle and during reload mode
4. **Blue LED** - Setup/special mode indicator - illuminates during startup, when no wood is present, and during reload mode

## Pin Assignments

### Motor Control Pins
- Cut Motor Pulse Pin: 48
- Cut Motor Direction Pin: 47
- Position Motor Pulse Pin: 21
- Position Motor Direction Pin: 20

### Switch and Sensor Pins
- Cut Motor Position Switch Pin: 10
- Position Motor Position Switch Pin: 9
- Reload Switch Pin: 14
- Cycle Switch Pin: 13
- YESorNO_WOOD_SENSOR Pin: 11
- WAS_WOOD_SUCTIONED_SENSOR Pin: 8

### Clamp Pins
- Position Clamp Pin: 18
- Wood Secure Clamp Pin: 17

### LED Pins
- Red LED Pin: 7
- Yellow LED Pin: 6
- Green LED Pin: 16
- Blue LED Pin: 15

### Signal Output
- Signal to Transfer Arm (TA) Pin: 19

## Key Parameters

### Motor Calibration
- Cut Motor: 63.5 steps per inch
- Position Motor: 1000 steps per inch
- Cut Motor Travel Distance: 8.5 inches
- Position Motor Travel Distance: 3.45 inches

### Motor Speeds
- Cut Motor Normal Speed: 80 steps/sec
- Cut Motor Return Speed: 2000 steps/sec
- Position Motor Normal Speed: 30000 steps/sec
- Position Motor Return Speed: 30000 steps/sec

### Motor Acceleration
- Cut Motor Acceleration: 2200 steps/sec²
- Position Motor Acceleration: 30000 steps/sec²
- Position Motor Return Acceleration: 30000 steps/sec²

### Homing Speeds
- Cut Motor Homing Speed: 300 steps/sec
- Position Motor Homing Speed: 2000 steps/sec

### Clamp Operation
- LOW signal = Clamp Extended/Engaged
- HIGH signal = Clamp Retracted

## System States

### 1. STARTUP_STATE
- Initial state when system powers on
- Initializes all pins and system configurations
- Transitions directly to HOMING_STATE

### 2. HOMING_STATE
- Runs the homing sequence to establish reference positions for both motors
- Blue LED blinks at 2-second intervals during this process
- Transitions to READY_STATE when complete

### 3. READY_STATE
- System waits for user input
- Green LED illuminates to indicate ready status
- Monitors Cycle Switch and Reload Switch for user actions
- Moves to RELOAD_STATE when Reload Switch is activated
- Moves to CUTTING_STATE when Cycle Switch is activated

### 4. RELOAD_STATE
- Accessible only from the READY_STATE
- Retracts both clamps to allow wood loading/unloading
- Blue LED illuminates to indicate reload mode
- Monitors Reload Switch for deactivation
- Must return to READY_STATE when Reload Switch is released before any cutting operation can begin
- Cannot directly transition to CUTTING_STATE as a safety measure

### 5. CUTTING_STATE
- Performs the cutting operation
- Verifies both clamps are properly engaged
- Moves cut motor to the cutting position (8.5 inches)
- Checks WAS_WOOD_SUCTIONED_SENSOR exactly once at 0.3 inches into the cut
- Signals the Transfer Arm (TA) when cut motor reaches 7.2 inches (0.5 inches before end of travel)
- Checks YESorNO_WOOD_SENSOR at the very end of the cutting movement
- Transitions to YESWOOD_STATE or NOWOOD_STATE based on YESorNO_WOOD_SENSOR reading

### 6. YESWOOD_STATE
- Handles sequence when wood is present for further cutting
- Returns both motors to home position
- Moves position motor to final position for next cut
- Manages clamp engagement/retraction during the sequence
- Returns to READY_STATE when complete

### 7. NOWOOD_STATE
- Handles sequence when no wood remains for cutting
- Returns both motors to home position
- Sets the isNoWoodCycleCompleted flag
- Returns to READY_STATE when complete

### 8. ERROR_STATE
- Manages various error conditions
- Displays distinct LED patterns based on error type
- Error is reset by toggling the Cycle Switch OFF and then ON
- Returns to HOMING_STATE when reset
- Immediately begins returning motors to position zero (does not halt in place)
- Keeps all clamps extended (engaged) to maintain secure grip on wood
- Moves at a safe, controlled speed during error recovery

## Error States

The system enters the ERROR_STATE due to specific detected conditions:

### 1. WOOD_SUCTION_ERROR_STATE
- Triggered when: WAS_WOOD_SUCTIONED_SENSOR indicates failure (Active LOW) at exactly the 0.3-inch point in the cut
- Visual indicator: Red LED blinks in pattern of 3 quick flashes followed by a pause (3 flashes, 1-second pause, repeat)
- Resolution: Operator must check vacuum system and remove any debris or blockage
- System behavior: Immediately begins returning cut motor to position zero while maintaining clamp extension
- Recovery: 
  1. System waits until Cycle Switch is toggled OFF
  2. When Cycle Switch is turned ON again, system returns to HOMING_STATE
  3. System performs the same safety check as during startup
  4. Normal operation resumes only after successful homing

### 2. CUT_MOTOR_HOME_ERROR_STATE
- Triggered when: Cut motor position switch does not read HIGH after motor completes return to home position
- Visual indicator: Red LED blinks in pattern of 2 quick flashes followed by a pause (2 flashes, 1-second pause, repeat)
- Possible causes: Mechanical obstruction, motor step loss, or switch failure
- System behavior: Attempted automatic recovery to reach position zero
- Recovery sequence:
  1. First attempt: Cut motor moves 1 inch forward then attempts to rehome (return to zero)
  2. If first rehoming fails: Motor moves forward 1 inch once more and tries rehoming again
  3. If second rehoming fails: System stops completely and requires a full power reset
  4. During this error state, the system will not respond to any switch inputs
  5. Position clamp and wood secure clamp remain extended to maintain secure grip on wood

### Error Recovery Flow
1. When any error occurs, the system immediately transitions to the appropriate error state
2. The red LED begins blinking in the pattern specific to the error type
3. The system immediately begins returning motors to position zero at a safe speed
4. The system waits for operator intervention
5. For WOOD_SUCTION_ERROR_STATE: Toggle Cycle Switch OFF then ON
6. For CUT_MOTOR_HOME_ERROR_STATE after second failure: Perform a full power reset
7. After recovery action, system returns to HOMING_STATE to re-establish reference positions
8. Normal operation resumes only after successful homing

## Cutting Cycle Sequence

The cutting cycle follows these specific steps:

1. **Cycle Start**: Initialize cycle variables and turn on yellow LED
2. **Clamp Verification**: Confirm both position and secure wood clamps are extended
3. **Cut Operation**: Move cut motor to 8.5-inch position
4. **Wood Suction Check**: Check WAS_WOOD_SUCTIONED_SENSOR when the cut motor reaches exactly 0.3 inches into the cut. If sensor indicates a problem (is LOW), immediately trigger an error condition and halt the cutting operation.
5. **Signal Transfer Arm**: Send signal to the Transfer Arm (TA) when the cut motor reaches 7.2 inches (0.5 inches before end of travel)
6. **YESorNO_WOOD_SENSOR Detection**: Check if wood is present using the YESorNO_WOOD_SENSOR for subsequent operations

For wood present (YESWOOD_STATE):
7. **Initial Return**: Start both motors moving to their home positions
8. **Initial Clamp Management**: Retract the position clamp prior to returning home, while keeping wood secure clamp extended
9. **Complete Return**: Continue movement until both motors reach their home position
10. **Home Position Check**: Wait 50ms after the cut motor reaches position zero, then verify the cut motor position switch reads HIGH. If not, trigger the CUT_MOTOR_HOME_ERROR_STATE
11. **Position Clamp Engagement**: Extend position clamp once position zero is reached
12. **Positioning Operation**: Move position motor to 3.45 inches for next cut, ensuring it first returns to zero
13. **Wood Clamp Verification**: Confirm wood secure clamp remains extended
14. **Cycle Complete**: Return to READY_STATE

For no wood present (NOWOOD_STATE):
7. **Clamp Management**: Retract wood secure clamp
8. **Home Return**: Both motors return to home position
9. **Position Clamp Release**: Retract position clamp
10. **Cycle Complete**: Set no-wood flag and return to READY_STATE. The system still allows one final cut to be performed before NOWOOD_STATE movements are executed

## Homing Sequence

The homing sequence establishes reference positions for both motors:

1. If position switches are already triggered, motors move away by 1 inch first
2. Motors move toward their respective home position switches at homing speed
3. Motors stop immediately when switches are activated and positions are set to 0
4. Position motor explicitly returns to zero before moving to operating position (3.45 inches)
5. System is now homed and ready for operation

## Operation Logic

- System performs homing sequence immediately after startup
- In READY_STATE, system monitors Cycle Switch and Reload Switch inputs
- RELOAD_STATE is only accessible from READY_STATE by activating the Reload Switch
- Both clamps remain retracted during RELOAD_STATE to facilitate wood handling
- System must return to READY_STATE from RELOAD before starting a cutting cycle
- This safety feature prevents accidental cycle activation during material loading
- WAS_WOOD_SUCTIONED_SENSOR is checked exactly once at 0.3 inches into the cut
- Transfer Arm signal is sent precisely at 7.2 inches of cut motor travel
- YESorNO_WOOD_SENSOR is checked at the very end of the cutting movement
- Machine continues cycling if Cycle Switch remains activated until NOWOOD_STATE is completed
- NOWOOD_STATE still allows one final cut to be performed before executing NOWOOD_STATE movements
- Safety check prevents operation if Cycle Switch is ON during startup
- Error detection continuously monitors for suction and homing failures
- WAS_WOOD_SUCTIONED_SENSOR monitoring detects improper cutting conditions early in the process
- YESorNO_WOOD_SENSOR determines if the machine should continue to the YESWOOD_STATE or NOWOOD_STATE

## Safety Features

- Homing sequence ensures accurate reference positions for reliable operation
- Switch debouncing prevents false readings and improper activations
- Both clamps must be engaged before cutting begins
- WAS_WOOD_SUCTIONED_SENSOR monitoring detects improper cutting conditions early in the process
- Cycle Switch safety check prevents unexpected operation at startup
- RELOAD_STATE requires returning to READY_STATE before cutting operations can begin
- Clear LED indicators provide visual feedback of system status

## Non-Blocking Operation

- System uses non-blocking delays via the Wait() function
- Continuous monitoring of switches occurs during all delay periods
- Motor operations execute without blocking the main control loop 

## Code Implementation Requirements

### Libraries
- **AccelStepper Library** - Required for all motor control operations
- **Bounce2 Library** - Required for debouncing all switches and sensors

### Debounce Configuration
- All switches and sensors must use a 10ms debounce time
- This includes:
  - Cut Motor Position Switch
  - Position Motor Position Switch
  - Reload Switch
  - Cycle Switch
  - YESorNO_WOOD_SENSOR
  - WAS_WOOD_SUCTIONED_SENSOR 

## State Management

### System States Overview
The following states must be implemented for complete system operation:

1. **STARTUP_STATE**: Initial state when power is applied
   - Initializes hardware, variables, and configurations
   - Performs initial safety checks
   - Transitions to: HOMING_STATE

2. **HOMING_STATE**: Establishes reference positions for motors
   - Handles motor movement to home position
   - Validates home position switches
   - Transitions to: READY_STATE

3. **READY_STATE**: System waiting for user input
   - Main idle state with green LED indicator
   - Monitors user input switches
   - Transitions to: RELOAD_STATE or CUTTING_STATE

4. **RELOAD_STATE**: Wood loading/unloading mode
   - Manages clamp retraction for wood handling
   - Monitors reload switch deactivation
   - Transitions to: READY_STATE only

5. **CUTTING_STATE**: Active cutting operation
   - Controls cut motor movement to 8.5 inches
   - Monitors sensors during cut
   - Transitions to: YESWOOD_STATE, NOWOOD_STATE, or ERROR_STATE

6. **YESWOOD_STATE**: Post-cut with wood remaining
   - Returns motors to home position
   - Manages position motor placement for next cut
   - Transitions to: READY_STATE

7. **NOWOOD_STATE**: Post-cut with no wood remaining
   - Returns motors to home position
   - Sets no-wood cycle flag
   - Transitions to: READY_STATE

8. **ERROR_STATE**: Error condition management
   - Handles error visualization via LED patterns
   - Returns motors to safe positions
   - Manages recovery operations
   - Transitions to: HOMING_STATE after reset

9. **CUT_MOTOR_HOME_ERROR_STATE**: Special error handling
   - Manages cut motor home position recovery
   - Attempts automatic rehoming
   - Transitions to: HOMING_STATE or requires power reset

10. **WOOD_SUCTION_ERROR_STATE**: Special error handling
    - Triggered by suction failure
    - Returns motors to zero position
    - Transitions to: HOMING_STATE after cycle switch toggle

### State Transition Table

| Current State | Condition | Next State |
|---------------|-----------|------------|
| STARTUP_STATE | After initialization | HOMING_STATE |
| HOMING_STATE | Homing complete | READY_STATE |
| READY_STATE | Reload Switch ON | RELOAD_STATE |
| READY_STATE | Cycle Switch ON | CUTTING_STATE |
| RELOAD_STATE | Reload Switch OFF | READY_STATE |
| CUTTING_STATE | Cut complete, wood present | YESWOOD_STATE |
| CUTTING_STATE | Cut complete, no wood | NOWOOD_STATE |
| CUTTING_STATE | WAS_WOOD_SUCTIONED_SENSOR LOW at 0.3" | WOOD_SUCTION_ERROR_STATE |
| YESWOOD_STATE | After processing | READY_STATE |
| NOWOOD_STATE | After processing | READY_STATE |
| ERROR_STATE | Cycle Switch toggle | HOMING_STATE |
| WOOD_SUCTION_ERROR_STATE | Cycle Switch toggle | HOMING_STATE |
| CUT_MOTOR_HOME_ERROR_STATE | After 2 failed attempts | Requires power reset |
| CUT_MOTOR_HOME_ERROR_STATE | Successful recovery | HOMING_STATE | 

## Implementation Functions

The following functions should be implemented to create a complete control system:

### 1. State Management Functions
- `void loop()` - Arduino/ESP32 main loop function that calls updateStateMachine()
- `void updateStateMachine()` - Main state machine control function
- `void enterState(State newState)` - Handles state transitions and initialization
- `bool isStateComplete()` - Checks if current state processing is complete
- `void resetStateVariables()` - Resets state-specific variables when entering a new state

### 2. Startup and Initialization
- `void setup()` - Arduino/ESP32 initialization function
- `void initializePins()` - Sets up pin modes and initial states
- `void configureMotors()` - Configures motor speed, acceleration parameters
- `void initializeDebounce()` - Sets up switch debouncing

### 3. Motor Control Functions
- `void homeCutMotor()` - Homes the cut motor to position zero
- `void homePositionMotor()` - Homes the position motor to position zero
- `void moveMotorToPosition(AccelStepper& motor, float targetPosition, float speed)` - Moves specified motor
- `void moveCutMotorToPosition(float position)` - Positions cut motor
- `void movePositionMotorToPosition(float position)` - Positions position motor 
- `void stopAllMotors()` - Immediately stops all motor movements
- `bool isMotorInPosition(AccelStepper& motor, float targetPosition)` - Checks if motor has reached target
- `void setMotorSpeeds(int cutSpeed, int positionSpeed)` - Updates motor speeds
- `void runMotors()` - Called in loop() to handle motor movements without blocking

### 4. Clamp Control Functions
- `void extendPositionClamp()` - Extends the position clamp
- `void retractPositionClamp()` - Retracts the position clamp
- `void extendWoodSecureClamp()` - Extends the wood secure clamp
- `void retractWoodSecureClamp()` - Retracts the wood secure clamp
- `bool areClampsPropertyExtended()` - Verifies both clamps are extended
- `bool areClampsPropertyRetracted()` - Verifies both clamps are retracted

### 5. Sensor Reading Functions
- `bool readCutMotorPositionSwitch()` - Reads cut motor position switch
- `bool readPositionMotorPositionSwitch()` - Reads position motor position switch
- `bool readReloadSwitch()` - Reads reload switch
- `bool readCycleSwitch()` - Reads cycle switch
- `bool isWoodPresent()` - Reads YESorNO_WOOD_SENSOR
- `bool isWoodSuctionProper()` - Reads WAS_WOOD_SUCTIONED_SENSOR
- `void updateAllSwitches()` - Updates all debounced switch readings

### 6. LED Control Functions
- `void setLEDState(int ledPin, bool state)` - Sets LED on/off
- `void updateRedLEDErrorPattern(int errorType)` - Controls red LED error pattern
- `void updateBlueLEDBlinking()` - Controls blue LED blinking during homing
- `void updateLEDsForState(State currentState)` - Updates LEDs based on current state

### 7. Homing Functions
- `void runHomingSequence()` - Full homing sequence for both motors
- `bool attemptCutMotorRehome()` - Used during error recovery
- `bool checkHomeSwitches()` - Verifies home switches are functioning

### 8. Error Handling Functions
- `void handleWoodSuctionError()` - Handles WAS_WOOD_SUCTIONED_SENSOR errors
- `void handleCutMotorHomeError()` - Handles home position errors
- `bool attemptErrorRecovery(ErrorType errorType)` - Attempts to recover from errors
- `void returnMotorsToSafePosition()` - Safely returns motors to zero position

### 9. Cutting Cycle Functions
- `void startCuttingCycle()` - Initiates cutting operation
- `void runYesWoodSequence()` - Handles sequence when wood is present
- `void runNoWoodSequence()` - Handles sequence when no wood is present
- `void signalTransferArm(bool state)` - Signals the transfer arm at the appropriate time
- `bool checkSuctionAtTargetPosition()` - Checks suction at the 0.3 inch position

### 10. Safety Check Functions
- `bool performStartupSafetyCheck()` - Checks if cycle switch is on during startup
- `bool isSystemReadyForCutting()` - Verifies all conditions for cutting are met

### 11. Utility Functions
- `bool Wait(unsigned long delayTime, unsigned long* startTimePtr)` - Non-blocking delay
- `float inchesToSteps(float inches, float stepsPerInch)` - Converts inches to motor steps
- `float stepsToInches(long steps, float stepsPerInch)` - Converts motor steps to inches
- `void logSystemStatus()` - Logs system status for debugging (Serial)
- `void updateSystemTiming()` - Manages system timing variables

## Constants and Enumerations

### State Enumeration
```cpp
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
```

### Error Type Enumeration
```cpp
enum ErrorType {
  NO_ERROR,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR
};
```

### Key Constants
```cpp
// Motor calibration constants
const float CUT_MOTOR_STEPS_PER_INCH = 63.5;
const float POSITION_MOTOR_STEPS_PER_INCH = 1000.0;

// Travel distances
const float CUT_MOTOR_TRAVEL_DISTANCE = 8.5;  // inches
const float POSITION_MOTOR_TRAVEL_DISTANCE = 3.45;  // inches

// Motor speeds
const float CUT_MOTOR_NORMAL_SPEED = 80.0;  // steps/sec
const float CUT_MOTOR_RETURN_SPEED = 2000.0;  // steps/sec
const float POSITION_MOTOR_NORMAL_SPEED = 30000.0;  // steps/sec
const float POSITION_MOTOR_RETURN_SPEED = 30000.0;  // steps/sec

// Motor acceleration
const float CUT_MOTOR_ACCELERATION = 2200.0;  // steps/sec²
const float POSITION_MOTOR_ACCELERATION = 30000.0;  // steps/sec²
const float POSITION_MOTOR_RETURN_ACCELERATION = 30000.0;  // steps/sec²

// Homing speeds
const float CUT_MOTOR_HOMING_SPEED = 300.0;  // steps/sec
const float POSITION_MOTOR_HOMING_SPEED = 2000.0;  // steps/sec

// Specific positions
const float WOOD_SUCTION_CHECK_POSITION = 0.3;  // inches
const float TRANSFER_ARM_SIGNAL_POSITION = 7.2;  // inches
```

### Pin Definitions
```cpp
// Motor control pins
const int CUT_MOTOR_PULSE_PIN = 48;
const int CUT_MOTOR_DIR_PIN = 47;
const int POSITION_MOTOR_PULSE_PIN = 21;
const int POSITION_MOTOR_DIR_PIN = 20;

// Switch and sensor pins
const int CUT_MOTOR_POSITION_SWITCH_PIN = 10;
const int POSITION_MOTOR_POSITION_SWITCH_PIN = 9;
const int RELOAD_SWITCH_PIN = 14;
const int CYCLE_SWITCH_PIN = 13;
const int YES_OR_NO_WOOD_SENSOR_PIN = 11;
const int WAS_WOOD_SUCTIONED_SENSOR_PIN = 8;

// Clamp pins
const int POSITION_CLAMP_PIN = 18;
const int WOOD_SECURE_CLAMP_PIN = 17;

// LED pins
const int RED_LED_PIN = 7;
const int YELLOW_LED_PIN = 6;
const int GREEN_LED_PIN = 16;
const int BLUE_LED_PIN = 15;

// Signal output
const int SIGNAL_TO_TRANSFER_ARM_PIN = 19;
```

### Timing Constants
```cpp
// Timing constants
const unsigned long DEBOUNCE_TIME = 10;        // ms for switch debouncing
const unsigned long BLUE_LED_BLINK_INTERVAL = 2000; // ms for blue LED blink during homing
const unsigned long RED_LED_ERROR_FLASH_ON = 200;   // ms for error LED flash on time
const unsigned long RED_LED_ERROR_FLASH_OFF = 200;  // ms for error LED flash off time
const unsigned long RED_LED_ERROR_PAUSE = 1000;     // ms for pause after error pattern
const unsigned long HOME_POSITION_CHECK_DELAY = 50; // ms to wait before checking home position
const unsigned long MOTOR_UPDATE_INTERVAL = 5;      // ms between motor updates in non-blocking mode
```

## Global Variables and State Tracking

### Essential Variable Declarations
```cpp
// State management
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;

// Motor instances
AccelStepper cutMotor(AccelStepper::DRIVER, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(AccelStepper::DRIVER, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

// Switch debouncing
Bounce cutMotorPositionSwitch = Bounce();
Bounce positionMotorPositionSwitch = Bounce();
Bounce reloadSwitch = Bounce();
Bounce cycleSwitch = Bounce();
Bounce yesOrNoWoodSensor = Bounce();
Bounce wasWoodSuctionedSensor = Bounce();

// Timing variables
unsigned long stateStartTime = 0;
unsigned long ledBlinkTime = 0;
unsigned long subStateTimer = 0;

// Flags
bool isHomingComplete = false;
bool isNoWoodCycleCompleted = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool hasWoodSensorBeenChecked = false;
bool isCycleInProgress = false;

// State tracking
int homingAttemptCount = 0;
int subState = 0;  // For tracking steps within states
```

## Implementation Templates

### State Machine Implementation
```cpp
void loop() {
  // Update all switch/sensor readings
  updateAllSwitches();
  
  // Run motors (non-blocking)
  runMotors();
  
  // Update state machine
  updateStateMachine();
  
  // Update LEDs based on current state
  updateLEDsForState(currentState);
}

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
    // Handle exit actions for each state
  }
  
  // Update state
  currentState = newState;
  stateStartTime = millis();
  subState = 0;
  
  // Entry actions for new state
  switch (currentState) {
    case STARTUP_STATE:
      // Initialize for startup
      break;
      
    case HOMING_STATE:
      // Start homing sequence
      break;
      
    // Handle entry actions for each state
  }
}
```

### Example State Handler (Cutting State)
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

## Testing Strategy

### Unit Test Plan
1. **Switch/Sensor Tests**
   - Test each switch with debouncing to verify correct readings
   - Validate each sensor functions correctly

2. **Motor Tests**
   - Verify each motor moves to a specified position
   - Test homing sequence for reliability
   - Validate acceleration/deceleration parameters

3. **Clamp Tests**
   - Verify clamp extension/retraction
   - Test clamp sequencing in each scenario

4. **Error Tests**
   - Simulate each error condition to verify error handling
   - Validate error recovery procedures
   - Test error LED patterns

5. **Full Cycle Tests**
   - Validate complete cutting cycle
   - Test YESWOOD and NOWOOD scenarios
   - Verify proper transitions between states

### Mock Hardware Testing
For development without hardware:
```cpp
// Mock hardware readings
bool mockSwitchValues[6] = {false, false, false, false, false, true};  // Default values
float mockMotorPositions[2] = {0.0, 0.0};  // [cutMotor, positionMotor] in inches

// Example mock function
bool readCutMotorPositionSwitch() {
  #ifdef MOCK_HARDWARE
    return mockSwitchValues[0];
  #else
    return cutMotorPositionSwitch.read();
  #endif
}

// Mock motor position
float getMotorPosition(int motorIndex) {
  #ifdef MOCK_HARDWARE
    return mockMotorPositions[motorIndex];
  #else
    if (motorIndex == 0) {
      return stepsToInches(cutMotor.currentPosition(), CUT_MOTOR_STEPS_PER_INCH);
    } else {
      return stepsToInches(positionMotor.currentPosition(), POSITION_MOTOR_STEPS_PER_INCH);
    }
  #endif
}
```

// ... existing code ... 