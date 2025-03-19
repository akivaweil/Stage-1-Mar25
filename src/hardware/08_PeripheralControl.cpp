#include "../../include/hardware/08_PeripheralControl.h"
#include "../../include/core/03_Utilities.h"

// Global variable for transfer arm signal timing
unsigned long transferArmSignalStartTime = 0;
bool transferArmSignalActive = false;

// Function to signal the transfer arm (non-blocking 500ms HIGH pulse)
void signalTransferArm(bool state) {
  if (state == HIGH && !transferArmSignalActive) {
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, HIGH);
    transferArmSignalStartTime = millis();
    transferArmSignalActive = true;
  } else if (state == LOW) {
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
    transferArmSignalActive = false;
  }
}

// Function to update transfer arm signal timing (call this in loop)
void updateTransferArmSignal() {
  if (transferArmSignalActive) {
    if (Wait(500, &transferArmSignalStartTime)) {
      digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
      transferArmSignalActive = false;
    }
  }
}

// Clamp control functions - all operations are immediate with no delays
void extendPositionClamp() {
  // LOW signal = Clamp Extended/Engaged
  digitalWrite(POSITION_CLAMP_PIN, LOW);
}

void retractPositionClamp() {
  // HIGH signal = Clamp Retracted
  digitalWrite(POSITION_CLAMP_PIN, HIGH);
}

void extendWoodSecureClamp() {
  // LOW signal = Clamp Extended/Engaged
  digitalWrite(WOOD_SECURE_CLAMP_PIN, LOW);
}

void retractWoodSecureClamp() {
  // HIGH signal = Clamp Retracted
  digitalWrite(WOOD_SECURE_CLAMP_PIN, HIGH);
}

// Function to initialize all pins
void initializePins() {
  // Configure motor control pins as outputs
  pinMode(CUT_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(CUT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_PULSE_PIN, OUTPUT);
  pinMode(POSITION_MOTOR_DIR_PIN, OUTPUT);
  
  // Configure switch and sensor pins with appropriate pull-up/down resistors
  pinMode(CUT_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLDOWN);    // Active HIGH
  pinMode(POSITION_MOTOR_HOMING_SWITCH_PIN, INPUT_PULLDOWN); // Active HIGH
  pinMode(RELOAD_SWITCH_PIN, INPUT_PULLDOWN);                // Active HIGH
  pinMode(CYCLE_SWITCH_PIN, INPUT_PULLDOWN);                 // Active HIGH
  pinMode(YES_OR_NO_WOOD_SENSOR_PIN, INPUT_PULLUP);          // Active LOW
  pinMode(WAS_WOOD_SUCTIONED_SENSOR_PIN, INPUT_PULLUP);      // Active LOW (but reads HIGH when proper)
  
  // Configure output pins
  pinMode(POSITION_CLAMP_PIN, OUTPUT);
  pinMode(WOOD_SECURE_CLAMP_PIN, OUTPUT);
  pinMode(SIGNAL_TO_TRANSFER_ARM_PIN, OUTPUT);
  
  // Configure LED pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  // Initialize outputs to default states
  digitalWrite(POSITION_CLAMP_PIN, HIGH);           // Retracted
  digitalWrite(WOOD_SECURE_CLAMP_PIN, HIGH);        // Retracted
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);    // Inactive
  
  // Initialize all LEDs to off
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
} 