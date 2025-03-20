#include "../../include/core/01_CommonDefinitions.h"
#include "../../include/core/02_PinDefinitions.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/core/00_WifiAndOTA.h"

#include "../../include/hardware/05_MotorControl.h"
#include "../../include/hardware/06_LEDControl.h"
#include "../../include/hardware/07_SwitchSensor.h"
#include "../../include/hardware/08_PeripheralControl.h"

#include "../../include/operations/09_StateMachine.h"
#include "../../include/operations/11_CuttingOperations.h"
#include "../../include/operations/12_YesWoodOperations.h"
#include "../../include/operations/13_NoWoodOperations.h"

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
  
  // Update WiFi and OTA only in READY_STATE
  updateWiFiAndOTA();
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
  
  // Initialize WiFi settings (but don't connect yet)
  initializeWiFi();
  
  // Perform startup safety check
  performStartupSafetyCheck();
  
  // Set initial state
  enterState(STARTUP_STATE);
} 