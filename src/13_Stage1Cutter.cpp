#include "core/01_CommonDefinitions.h"
#include "core/02_PinDefinitions.h"
#include "core/03_Utilities.h"

#include "hardware/05_MotorControl.h"
#include "hardware/06_LEDControl.h"
#include "hardware/07_SwitchSensor.h"
#include "hardware/08_PeripheralControl.h"

#include "operations/09_StateMachine.h"
#include "operations/11_CuttingOperations.h"
#include "operations/12_YesWoodOperations.h"
#include "operations/13_NoWoodOperations.h"

// Global state variables
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;
int subState = 0;
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;

/**
 * Main loop - runs continuously after setup
 */
void loop() {
  // Update switches
  updateAllSwitches();
  
  // Run motors
  runMotors();
  
  // Signal transfer arm if needed
  signalTransferArm(hasTransferArmBeenSignaled);
  
  // Update state machine
  updateStateMachine();
  
  // Update LEDs based on current state
  updateAllLEDs();
}

/**
 * Setup - runs once at startup
 */
void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  
  // Initialize pins
  initializePins();
  
  // Initialize debounce
  initializeDebounce();
  
  // Configure motors
  configureMotors();
  
  // Perform startup safety check
  performStartupSafetyCheck();
  
  // Set initial state
  enterState(STARTUP_STATE);
} 