# Operation Logic

This document describes the overall operational logic of the wood cutting machine.

## General Operation Flow

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

## Timing Management

```cpp
// Non-blocking delay function
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
```

## State Variables and Flags

```cpp
// State tracking variables
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;
unsigned long stateStartTime = 0;
int subState = 0;

// Operation flags
bool isHomingComplete = false;
bool isNoWoodCycleCompleted = false;
bool isCutMotorHomed = false;
bool isPositionMotorHomed = false;
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool hasWoodSensorBeenChecked = false;
bool isCycleInProgress = false;
``` 