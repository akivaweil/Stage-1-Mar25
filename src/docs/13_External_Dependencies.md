# External Dependencies

This document describes the external libraries required for the wood cutting machine control system, along with installation and configuration details.

## Required Libraries

### AccelStepper Library

#### Purpose
The AccelStepper library is essential for:
- Controlling stepper motors with acceleration/deceleration profiles
- Non-blocking motor movement
- Precise position control

#### Features Used
- Driver mode for controlling stepper drivers
- Position tracking and setting
- Speed and acceleration control
- Non-blocking operation

#### Installation
1. In Arduino IDE: Sketch > Include Library > Manage Libraries...
2. Search for "AccelStepper"
3. Install version 1.61 or newer by Mike McCauley

In PlatformIO:
```
lib_deps =
    waspinator/AccelStepper @ ^1.61
```

#### Implementation Notes
- We use the DRIVER interface mode for both motors
- Max speed and acceleration should be set during setup
- Call `runMotors()` frequently in the main loop

### Bounce2 Library

#### Purpose
The Bounce2 library is used for:
- Debouncing all switches and sensors
- Preventing false readings due to switch bounce
- Providing stable input signals

#### Features Used
- Simple interface for debouncing digital inputs
- Consistent 10ms debounce time for all inputs
- Event detection (pressed/released)

#### Installation
1. In Arduino IDE: Sketch > Include Library > Manage Libraries...
2. Search for "Bounce2"
3. Install version 2.7.0 or newer by Thomas O. Fredericks

In PlatformIO:
```
lib_deps =
    thomasfredericks/Bounce2 @ ^2.7.0
```

#### Implementation Notes
- All switches and sensors must use a 10ms debounce time
- Attach each input in the setup function
- Call `update()` method for each switch in every loop iteration

## Library Configuration

### AccelStepper Configuration

```cpp
#include <AccelStepper.h>

// Create instances
AccelStepper cutMotor(AccelStepper::DRIVER, CUT_MOTOR_PULSE_PIN, CUT_MOTOR_DIR_PIN);
AccelStepper positionMotor(AccelStepper::DRIVER, POSITION_MOTOR_PULSE_PIN, POSITION_MOTOR_DIR_PIN);

void configureMotors() {
  // Configure cut motor
  cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
  cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
  
  // Configure position motor
  positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
  positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
}

// Non-blocking motor run function
void runMotors() {
  cutMotor.run();
  positionMotor.run();
}
```

### Bounce2 Configuration

```cpp
#include <Bounce2.h>

// Create instances
Bounce cutMotorPositionSwitch = Bounce();
Bounce positionMotorPositionSwitch = Bounce();
Bounce reloadSwitch = Bounce();
Bounce cycleSwitch = Bounce();
Bounce yesOrNoWoodSensor = Bounce();
Bounce wasWoodSuctionedSensor = Bounce();

void initializeDebounce() {
  // Attach and configure each switch
  cutMotorPositionSwitch.attach(CUT_MOTOR_POSITION_SWITCH_PIN, INPUT_PULLUP);
  cutMotorPositionSwitch.interval(DEBOUNCE_TIME);
  
  positionMotorPositionSwitch.attach(POSITION_MOTOR_POSITION_SWITCH_PIN, INPUT_PULLUP);
  positionMotorPositionSwitch.interval(DEBOUNCE_TIME);
  
  reloadSwitch.attach(RELOAD_SWITCH_PIN, INPUT_PULLUP);
  reloadSwitch.interval(DEBOUNCE_TIME);
  
  cycleSwitch.attach(CYCLE_SWITCH_PIN, INPUT_PULLUP);
  cycleSwitch.interval(DEBOUNCE_TIME);
  
  yesOrNoWoodSensor.attach(YES_OR_NO_WOOD_SENSOR_PIN, INPUT_PULLUP);
  yesOrNoWoodSensor.interval(DEBOUNCE_TIME);
  
  wasWoodSuctionedSensor.attach(WAS_WOOD_SUCTIONED_SENSOR_PIN, INPUT_PULLUP);
  wasWoodSuctionedSensor.interval(DEBOUNCE_TIME);
}

// Update all switches in each loop iteration
void updateAllSwitches() {
  cutMotorPositionSwitch.update();
  positionMotorPositionSwitch.update();
  reloadSwitch.update();
  cycleSwitch.update();
  yesOrNoWoodSensor.update();
  wasWoodSuctionedSensor.update();
}
```

## platformio.ini Configuration

Here's a complete configuration for PlatformIO:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    waspinator/AccelStepper @ ^1.61
    thomasfredericks/Bounce2 @ ^2.7.0
build_flags =
    -D ESP32
    ;-D DEBUG_MODE          ; Uncomment to enable debugging
    ;-D MOCK_HARDWARE       ; Uncomment for mock hardware testing
``` 