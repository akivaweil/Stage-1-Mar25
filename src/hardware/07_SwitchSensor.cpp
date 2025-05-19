#include "../../include/hardware/07_SwitchSensor.h"

// Constants
const unsigned long DEBOUNCE_TIME = 15; // ms for switch debouncing

// Switch debouncing instances
Bounce cutMotorHomingSwitch = Bounce();
Bounce positionMotorHomingSwitch = Bounce();
Bounce reloadSwitch = Bounce();
Bounce cycleSwitch = Bounce();
Bounce yesOrNoWoodSensor = Bounce();
Bounce wasWoodSuctionedSensor = Bounce();

// Global variables
bool prevCycleSwitchState = false;
bool needCycleSwitchToggle = false;

// Initialize switch debouncing with 15ms debounce time
void initializeDebounce() {
  cutMotorHomingSwitch.attach(CUT_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLDOWN);
  cutMotorHomingSwitch.interval(DEBOUNCE_TIME);
  
  positionMotorHomingSwitch.attach(POSITION_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLDOWN);
  positionMotorHomingSwitch.interval(DEBOUNCE_TIME);
  
  reloadSwitch.attach(RELOAD_SWITCH_PIN, INPUT_PULLDOWN);
  reloadSwitch.interval(DEBOUNCE_TIME);
  
  cycleSwitch.attach(CYCLE_SWITCH_PIN, INPUT_PULLDOWN);
  cycleSwitch.interval(DEBOUNCE_TIME);
  
  yesOrNoWoodSensor.attach(YES_OR_NO_WOOD_SENSOR_PIN, INPUT_PULLUP);
  yesOrNoWoodSensor.interval(DEBOUNCE_TIME);
  
  wasWoodSuctionedSensor.attach(WAS_WOOD_SUCTIONED_SENSOR_PIN, INPUT_PULLUP);
  wasWoodSuctionedSensor.interval(DEBOUNCE_TIME);
}

// Update all switch readings
void updateAllSwitches() {
  cutMotorHomingSwitch.update();
  positionMotorHomingSwitch.update();
  reloadSwitch.update();
  cycleSwitch.update();
  yesOrNoWoodSensor.update();
  wasWoodSuctionedSensor.update();
}

// Sensor reading functions 
bool readCutMotorHomingSwitch() {
  // Active HIGH
  return cutMotorHomingSwitch.read() == HIGH;
}

bool readPositionMotorHomingSwitch() {
  // Active HIGH
  return positionMotorHomingSwitch.read() == HIGH;
}

bool readReloadSwitch() {
  // Active HIGH
  return reloadSwitch.read() == HIGH;
}

bool readCycleSwitch() {
  // Active HIGH
  return cycleSwitch.read() == HIGH;
}

bool isWoodPresent() {
  // Active LOW - wood is present when sensor reads LOW
  return yesOrNoWoodSensor.read() == LOW;
}

bool isWoodSuctionProper() {
  // Active LOW - suction is proper when sensor reads HIGH (inverted logic)
  return wasWoodSuctionedSensor.read() == HIGH;
}

// Detects a cycle switch toggle (OFF to ON)
bool cycleToggleDetected() {
  bool currentCycleSwitchState = readCycleSwitch();
  bool result = (prevCycleSwitchState == false && currentCycleSwitchState == true);
  prevCycleSwitchState = currentCycleSwitchState;
  return result;
}

// Perform startup safety check - checks if cycle switch is on during startup
bool performStartupSafetyCheck() {
  // If cycle switch is ON at startup, require it to be toggled OFF then ON before allowing operation
  if (readCycleSwitch()) {
    needCycleSwitchToggle = true;
    return false;
  }
  
  needCycleSwitchToggle = false;
  return true;
} 