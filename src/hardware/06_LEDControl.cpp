#include "../../include/hardware/06_LEDControl.h"
#include "../../include/core/03_Utilities.h"

// LED Pattern Functions
void allLedsOff() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void setGreenLed(bool state) {
  digitalWrite(GREEN_LED_PIN, state ? HIGH : LOW);
}

void setBlueLed(bool state) {
  digitalWrite(BLUE_LED_PIN, state ? HIGH : LOW);
}

void setYellowLed(bool state) {
  digitalWrite(YELLOW_LED_PIN, state ? HIGH : LOW);
}

void setRedLed(bool state) {
  digitalWrite(RED_LED_PIN, state ? HIGH : LOW);
}

void setYesWoodPattern() {
  setGreenLed(true);
  setYellowLed(true);
  setBlueLed(false);
  setRedLed(false);
}

void setNoWoodPattern() {
  setBlueLed(true);
  setGreenLed(true);
  setYellowLed(false);
  setRedLed(false);
}

void updateBlueBlinkPattern() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;
  
  if (Wait(500, &lastBlinkTime)) {
    ledState = !ledState;
    setBlueLed(ledState);
  }
}

void setWoodSuctionErrorPattern() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;
  
  // Fast blinking for wood suction error: 200ms on, 200ms off
  if (Wait(200, &lastBlinkTime)) {
    ledState = !ledState;
    setRedLed(ledState);
  }
}

void setCutMotorHomeErrorPattern() {
  // Double pulse pattern for homing error: 200ms on, 200ms off, 200ms on, 600ms off
  unsigned long cycleTime = millis() % 1200;
  
  if (cycleTime < 200) {
    setRedLed(true);
  } else if (cycleTime < 400) {
    setRedLed(false);
  } else if (cycleTime < 600) {
    setRedLed(true);
  } else {
    setRedLed(false);
  }
}

void setPositionMotorHomeErrorPattern() {
  // Alternating red and blue LED pattern for position motor homing error
  unsigned long cycleTime = millis() % 1000;
  
  if (cycleTime < 500) {
    setRedLed(true);
    setBlueLed(false);
  } else {
    setRedLed(false);
    setBlueLed(true);
  }
}

void setGeneralErrorPattern() {
  setRedLed(true);
}

// Update the red LED error pattern based on error type
void updateRedLEDErrorPattern(ErrorType errorType) {
  switch (errorType) {
    case WOOD_SUCTION_ERROR:
      setWoodSuctionErrorPattern();
      break;
      
    case CUT_MOTOR_HOME_ERROR:
      setCutMotorHomeErrorPattern();
      break;
      
    case POSITION_MOTOR_HOME_ERROR:
      setPositionMotorHomeErrorPattern();
      break;
      
    case NO_ERROR:
    default:
      setGeneralErrorPattern();
      break;
  }
}

// Update the LEDs based on the current state
void updateLEDsForState(State currentState) {
  allLedsOff();
  
  switch (currentState) {
    case STARTUP_STATE:
      setBlueLed(true);
      break;
      
    case HOMING_STATE:
      updateBlueBlinkPattern();
      break;
      
    case READY_STATE:
      setGreenLed(true);
      break;
      
    case RELOAD_STATE:
      setBlueLed(true);
      break;
      
    case CUTTING_STATE:
      setYellowLed(true);
      break;
      
    case YESWOOD_STATE:
      setYesWoodPattern();
      break;
      
    case NOWOOD_STATE:
      setNoWoodPattern();
      break;
      
    case ERROR_STATE:
    case WOOD_SUCTION_ERROR_STATE:
    case CUT_MOTOR_HOME_ERROR_STATE:
    case POSITION_MOTOR_HOME_ERROR_STATE:
      // Error states LED patterns are handled by updateRedLEDErrorPattern()
      break;
  }
}

/**
 * Update all LEDs based on current state and error condition
 * Called from the main loop
 */
void updateAllLEDs() {
  // First update the state-based LEDs
  updateLEDsForState(currentState);
  
  // If we're in an error state, also update the error LED pattern
  if (currentState == ERROR_STATE || 
      currentState == WOOD_SUCTION_ERROR_STATE || 
      currentState == CUT_MOTOR_HOME_ERROR_STATE || 
      currentState == POSITION_MOTOR_HOME_ERROR_STATE) {
    updateRedLEDErrorPattern(currentError);
  }
}

// Blinking functions - use these for temporary indication, they don't keep state
// and only blink once when called

// Blink red LED
void blinkRedLed() {
  static unsigned long lastToggleTime = 0;
  static bool ledState = false;
  
  if (Wait(250, &lastToggleTime)) {
    ledState = !ledState;
    digitalWrite(RED_LED_PIN, ledState);
  }
}

// Blink yellow LED
void blinkYellowLed() {
  static unsigned long lastToggleTime = 0;
  static bool ledState = false;
  
  if (Wait(250, &lastToggleTime)) {
    ledState = !ledState;
    digitalWrite(YELLOW_LED_PIN, ledState);
  }
}

// Blink green LED
void blinkGreenLed() {
  static unsigned long lastToggleTime = 0;
  static bool ledState = false;
  
  if (Wait(250, &lastToggleTime)) {
    ledState = !ledState;
    digitalWrite(GREEN_LED_PIN, ledState);
  }
}

// Blink blue LED
void blinkBlueLed() {
  static unsigned long lastToggleTime = 0;
  static bool ledState = false;
  
  if (Wait(250, &lastToggleTime)) {
    ledState = !ledState;
    digitalWrite(BLUE_LED_PIN, ledState);
  }
} 