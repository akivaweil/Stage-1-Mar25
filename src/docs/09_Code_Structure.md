# Code Structure

This document outlines the functions needed to implement the wood cutting machine control system.

## Implementation Functions Overview

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

## Code Organization

The implementation should follow this organization:

```
// Include necessary libraries
#include <AccelStepper.h>
#include <Bounce2.h>

// Define constants and enumerations
enum State {...};
enum ErrorType {...};
const int PIN_DEFINITIONS...

// Declare global variables
AccelStepper cutMotor(...);
AccelStepper positionMotor(...);
Bounce switches...
State currentState = STARTUP_STATE;

// Setup function
void setup() {
  initializePins();
  configureMotors();
  initializeDebounce();
}

// Main loop
void loop() {
  updateAllSwitches();
  runMotors();
  updateStateMachine();
  updateLEDsForState(currentState);
}

// State management functions
void updateStateMachine() {...}
void enterState(State newState) {...}

// State handler functions
void handleStartupState() {...}
void handleHomingState() {...}
void handleReadyState() {...}
// etc.

// Motor control functions
void homeCutMotor() {...}
void homePositionMotor() {...}
// etc.

// Utility functions
bool Wait(unsigned long delayTime, unsigned long* startTimePtr) {...}
float inchesToSteps(float inches, float stepsPerInch) {...}
// etc.
```

## Library Dependencies

- **AccelStepper Library** - Required for all motor control operations
- **Bounce2 Library** - Required for debouncing all switches and sensors 