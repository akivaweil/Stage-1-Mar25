# Pin Assignments

This document defines the pin assignments for the wood cutting machine ESP32-S3 controller.

## ESP32-S3 Development Board Pin Layout (Freenove ESP32-S3)

**Left side (top to bottom):**
- 4, 5, 6, 7, 15, 16, 17, 18, 8, 3, 46, 9, 10, 11, 12, 13, 14

**Right side (top to bottom):**
- 1, 2, 42, 41, 40, 39, 38, 37, 36, 35, 0, 45, 48, 47, 21, 20, 19

## Input Pins

| Pin | Function | Type | Logic Level | Debounce | Notes |
|-----|----------|------|-------------|----------|-------|
| 10  | CUT_MOTOR_HOMING_SWITCH_PIN | Digital Input | Active HIGH | 15ms | HIGH when cut motor at home position |
| 9   | POSITION_MOTOR_HOMING_SWITCH_PIN | Digital Input | Active HIGH | 15ms | HIGH when position motor at home position |
| 14  | RELOAD_SWITCH_PIN | Digital Input | Active HIGH | 15ms | HIGH when reload switch activated |
| 13  | CYCLE_SWITCH_PIN | Digital Input | Active HIGH | 15ms | HIGH when cycle switch activated |
| 11  | YES_OR_NO_WOOD_SENSOR_PIN | Digital Input | Active LOW | 15ms | LOW when wood present |
| 8   | WAS_WOOD_SUCTIONED_SENSOR_PIN | Digital Input | Active LOW | 15ms | HIGH when suction proper |

## Output Pins

| Pin | Function | Type | Logic Level | Notes |
|-----|----------|------|-------------|-------|
| 21  | POSITION_MOTOR_PULSE_PIN | Digital Output | - | Step pulse pin for position motor driver |
| 20  | POSITION_MOTOR_DIR_PIN | Digital Output | - | Direction pin for position motor driver |
| 48  | CUT_MOTOR_PULSE_PIN | Digital Output | - | Step pulse pin for cut motor driver |
| 47  | CUT_MOTOR_DIR_PIN | Digital Output | - | Direction pin for cut motor driver |
| 18  | POSITION_CLAMP_PIN | Digital Output | Active LOW | LOW to extend (engage), HIGH to retract |
| 17  | WOOD_SECURE_CLAMP_PIN | Digital Output | Active LOW | LOW to extend (engage), HIGH to retract |
| 19  | SIGNAL_TO_TRANSFER_ARM_PIN | Digital Output | Active HIGH | HIGH pulse (500ms) to signal transfer arm |
| 7   | RED_LED_PIN | Digital Output | Active HIGH | HIGH to turn on red LED |
| 6   | YELLOW_LED_PIN | Digital Output | Active HIGH | HIGH to turn on yellow LED |
| 16  | GREEN_LED_PIN | Digital Output | Active HIGH | HIGH to turn on green LED |
| 15  | BLUE_LED_PIN | Digital Output | Active HIGH | HIGH to turn on blue LED |

## Pin Definitions in Code

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

// Debounce time
const unsigned long DEBOUNCE_TIME = 15; // ms for switch debouncing
```

## Pin Configuration

```cpp
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
```

## Debouncing Implementation

All input pins should be debounced to ensure reliable operation. The recommended debounce time is 15ms.

```cpp
// Debounce class for reliable switch reading
class Debounce {
private:
  uint8_t pin;
  unsigned long debounceTime;
  unsigned long lastDebounceTime;
  int lastStableState;
  int currentState;

public:
  Debounce(uint8_t pin, unsigned long debounceTime) 
    : pin(pin), debounceTime(debounceTime), lastDebounceTime(0), lastStableState(0), currentState(0) {}
  
  void begin() {
    lastStableState = digitalRead(pin);
    currentState = lastStableState;
  }
  
  int read() {
    int reading = digitalRead(pin);
    
    if (reading != currentState) {
      lastDebounceTime = millis();
      currentState = reading;
    }
    
    if ((millis() - lastDebounceTime) > debounceTime) {
      lastStableState = currentState;
    }
    
    return lastStableState;
  }
};

// Create debounce instances for each input
Debounce cutMotorHomingSwitch(CUT_MOTOR_HOMING_SWITCH_PIN, DEBOUNCE_TIME);
Debounce positionMotorHomingSwitch(POSITION_MOTOR_HOMING_SWITCH_PIN, DEBOUNCE_TIME);
Debounce reloadSwitch(RELOAD_SWITCH_PIN, DEBOUNCE_TIME);
Debounce cycleSwitch(CYCLE_SWITCH_PIN, DEBOUNCE_TIME);
Debounce yesOrNoWoodSensor(YES_OR_NO_WOOD_SENSOR_PIN, DEBOUNCE_TIME);
Debounce wasWoodSuctionedSensor(WAS_WOOD_SUCTIONED_SENSOR_PIN, DEBOUNCE_TIME);
```

## Reading Inputs with Logic Level Handling

```cpp
// Active HIGH sensors (read HIGH when activated)
bool isCutMotorAtHome() { return cutMotorHomingSwitch.read() == HIGH; }
bool isPositionMotorAtHome() { return positionMotorHomingSwitch.read() == HIGH; }
bool isReloadSwitchActivated() { return reloadSwitch.read() == HIGH; }
bool isCycleSwitchActivated() { return cycleSwitch.read() == HIGH; }

// Active LOW sensors
bool isWoodPresent() { return yesOrNoWoodSensor.read() == LOW; }  // Wood present when LOW
bool isWoodSuctionProper() { return wasWoodSuctionedSensor.read() == HIGH; }  // Proper when HIGH
```

## Controlling Outputs

```cpp
// Immediate operations - no delays
void extendPositionClamp() { digitalWrite(POSITION_CLAMP_PIN, LOW); }  // Extend = LOW
void retractPositionClamp() { digitalWrite(POSITION_CLAMP_PIN, HIGH); } // Retract = HIGH
void extendWoodSecureClamp() { digitalWrite(WOOD_SECURE_CLAMP_PIN, LOW); }  // Extend = LOW
void retractWoodSecureClamp() { digitalWrite(WOOD_SECURE_CLAMP_PIN, HIGH); } // Retract = HIGH

// Non-blocking transfer arm signal using the Wait function
bool isTransferArmSignalComplete(unsigned long* signalStartTime) {
  if (*signalStartTime == 0) return true; // No signal in progress
  
  if (Wait(500, signalStartTime)) {
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW); // Turn off signal after 500ms
    return true; // Signal complete
  }
  return false; // Signal still in progress
}

// Start transfer arm signal
void startTransferArmSignal(unsigned long* signalStartTime) {
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, HIGH);
  *signalStartTime = millis();
}
```

## LED Status Indicators

```cpp
// LED control functions
void setLedState(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

// Set state indicator LEDs based on current system state
void updateStatusLeds(int currentState) {
  // Turn all LEDs off first
  setLedState(RED_LED_PIN, false);
  setLedState(YELLOW_LED_PIN, false);
  setLedState(GREEN_LED_PIN, false);
  setLedState(BLUE_LED_PIN, false);
  
  // Set appropriate LED based on state
  switch (currentState) {
    case IDLE_STATE:
      setLedState(GREEN_LED_PIN, true);
      break;
      
    case HOMING_STATE:
      setLedState(BLUE_LED_PIN, true);
      break;
      
    case CUTTING_STATE:
      setLedState(YELLOW_LED_PIN, true);
      break;
      
    case ERROR_STATE:
      setLedState(RED_LED_PIN, true);
      break;
      
    case YESWOOD_STATE:
      setLedState(GREEN_LED_PIN, true);
      setLedState(YELLOW_LED_PIN, true); // Green + Yellow
      break;
      
    case NOWOOD_STATE:
      setLedState(BLUE_LED_PIN, true);
      setLedState(GREEN_LED_PIN, true); // Blue + Green
      break;
  }
}
```

## Notes on Peripheral Connection

- All switches should be connected between the ESP32-S3 pin and GND (for active HIGH with pulldown) or between the pin and 3.3V (for active LOW with pullup)
- LEDs must have appropriate current-limiting resistors
- Solenoid/relay drivers should have appropriate isolation (optocoupler or MOSFET)
- Motor driver connections require appropriate isolation and timing for the specific driver model