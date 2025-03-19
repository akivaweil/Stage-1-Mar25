# Stage 1 Cutting System

## Overview
This project is a control system for a Stage 1 cutting device using an ESP32-S3 microcontroller. The system controls two stepper motors, monitors various sensors, and provides visual feedback through LEDs.

## Hardware
- ESP32-S3 microcontroller
- Two stepper motors with drivers (cutting motor and positioning motor)
- Limit switches for homing
- Reload and cycle switches for user interaction
- Wood presence and suction sensors
- Position and wood secure clamps
- Signal output to transfer arm
- Four status LEDs (red, yellow, green, blue)

## Software Structure
The codebase has been modularized into the following components:

- **CommonDefinitions.h**: Common enums, constants, and globals
- **PinDefinitions**: Pin assignments for all hardware components
- **MotorControl**: Stepper motor configuration and movement
- **LEDControl**: LED status indicators and patterns
- **SwitchSensor**: Switch and sensor input handling
- **PeripheralControl**: Clamp and transfer arm signaling
- **StateMachine**: State transition logic and handling
- **CuttingOperations**: Cutting cycle functionality
- **YesWoodOperations**: Logic for when wood is detected
- **NoWoodOperations**: Logic for when no wood is detected
- **Utilities**: Helper functions like non-blocking delay

## State Machine
The system operates based on a state machine with the following states:
1. STARTUP_STATE
2. HOMING_STATE
3. READY_STATE
4. RELOAD_STATE
5. CUTTING_STATE
6. YESWOOD_STATE
7. NOWOOD_STATE
8. ERROR_STATE
9. WOOD_SUCTION_ERROR_STATE
10. CUT_MOTOR_HOME_ERROR_STATE
11. POSITION_MOTOR_HOME_ERROR_STATE

## Operation Flow
1. System starts up and enters homing state
2. After successful homing, system enters ready state
3. When cycle switch is toggled, cutting cycle begins
4. After cutting, system checks for wood presence
5. System indicates wood presence or absence with LEDs
6. System returns to ready state or reload state

## Error Handling
The system has built-in error detection and recovery for:
- Wood suction problems
- Cut motor homing failures
- Position motor homing failures

Error states are indicated with specific LED patterns and can be recovered by toggling the cycle switch.

## Dependencies
- AccelStepper library for stepper motor control
- Bounce2 library for switch debouncing 