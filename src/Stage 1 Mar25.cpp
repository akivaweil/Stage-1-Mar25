#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

#include "../include/CommonDefinitions.h"
#include "../include/PinDefinitions.h"
#include "../include/MotorControl.h"
#include "../include/LEDControl.h"
#include "../include/SwitchSensor.h"
#include "../include/PeripheralControl.h"
#include "../include/StateMachine.h"
#include "../include/CuttingOperations.h"
#include "../include/YesWoodOperations.h"
#include "../include/NoWoodOperations.h"
#include "../include/Utilities.h"

// Global state variables
State currentState = STARTUP_STATE;
ErrorType currentError = NO_ERROR;
int subState = 0;
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;

void loop() {
  updateAllSwitches();
  runMotors();
  updateTransferArmSignal();
  updateStateMachine();
  updateLEDsForState(currentState);
}

// Arduino setup function
void setup() {
  initializePins();
  initializeDebounce();
  configureMotors();
  performStartupSafetyCheck();
  currentState = STARTUP_STATE;
}
