# Constants and Variables

This document provides a comprehensive list of all constants, enumerations, and global variables used in the wood cutting machine control system.

## State Enumerations

```cpp
// System state enumeration
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

// Error type enumeration
enum ErrorType {
  NO_ERROR,
  WOOD_SUCTION_ERROR,
  CUT_MOTOR_HOME_ERROR,
  POSITION_MOTOR_HOME_ERROR
};
```

## Pin Definitions

```cpp
// Motor control pins
const int CUT_MOTOR_PULSE_PIN = 48;
const int CUT_MOTOR_DIR_PIN = 47;
const int POSITION_MOTOR_PULSE_PIN = 21;
const int POSITION_MOTOR_DIR_PIN = 20;

// Switch and sensor pins
const int CUT_MOTOR_HOMING_SWITCH_PIN = 10;
const int POSITION_MOTOR_HOMING_SWITCH_PIN = 9;
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

## Motor Configuration Constants

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
```

## Position Constants

```cpp
// Specific positions
const float WOOD_SUCTION_CHECK_POSITION = 0.3;  // inches
const float TRANSFER_ARM_SIGNAL_POSITION = 7.2;  // inches
const float CUT_COMPLETE_POSITION = 8.5;  // inches
const float HOMING_BACKUP_DISTANCE = 1.0;  // inches for moving away from switches
const float POSITION_MOTOR_RETRACT_DISTANCE = 0.1;  // inches
```

## Timing Constants

```cpp
// Timing constants
const unsigned long DEBOUNCE_TIME = 15;  // ms for switch debouncing
const unsigned long BLUE_LED_BLINK_INTERVAL = 2000;  // ms for blue LED blink during homing
const unsigned long RED_LED_ERROR_FLASH_ON = 200;  // ms for error LED flash on time
const unsigned long RED_LED_ERROR_FLASH_OFF = 200;  // ms for error LED flash off time
const unsigned long RED_LED_ERROR_PAUSE = 1000;  // ms for pause after error pattern
const unsigned long HOME_POSITION_CHECK_DELAY = 50;  // ms to wait before checking home position
const unsigned long MOTOR_UPDATE_INTERVAL = 5;  // ms between motor updates in non-blocking mode
```

## Global Variables

### Motor and Sensor Objects

```cpp
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
```

### State Management Variables

```cpp
// State tracking
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;
int subState = 0;  // For tracking steps within states

// Timing variables
unsigned long stateStartTime = 0;
unsigned long ledBlinkTime = 0;
unsigned long subStateTimer = 0;
```

### Status Flags

```cpp
// Operation flags
bool isHomingComplete = false;
bool isNoWoodCycleCompleted = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool hasWoodSensorBeenChecked = false;
bool isCycleInProgress = false;

// Error recovery tracking
int homingAttemptCount = 0;
```

## Preprocessor Directives

```cpp
// Debug mode - uncomment to enable Serial debugging
//#define DEBUG_MODE

// Mock hardware - uncomment during development without hardware
//#define MOCK_HARDWARE

// Safety options
#define ENABLE_STARTUP_SAFETY_CHECK
#define MAX_HOMING_ATTEMPTS 2
``` 