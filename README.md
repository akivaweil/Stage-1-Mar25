# Stage 1 Cutter

This project controls a modular cutting system using an ESP32-S3 microcontroller. It manages stepper motors, sensors, and LEDs to perform automated cutting cycles.

## Project Structure

The codebase is organized into modular components:

### Core
- `01_CommonDefinitions.h` - Enumerations, constants, and global variable declarations
- `02_PinDefinitions.h` - Pin definitions for all hardware components
- `03_Utilities.h` - Utility functions like non-blocking delays

### Hardware
- `05_MotorControl.h/cpp` - Motor control functions
- `06_LEDControl.h/cpp` - LED control and patterns
- `07_SwitchSensor.h/cpp` - Switch and sensor handling
- `08_PeripheralControl.h/cpp` - Peripheral control (clamps, transfer arm)

### Operations
- `09_StateMachine.h/cpp` - Main state machine implementation
- `10_CuttingOperations.h/cpp` - Cutting cycle operations
- `11_YesWoodOperations.h/cpp` - Wood present operations
- `12_NoWoodOperations.h/cpp` - No wood operations

## Main Application
- `13_Stage1Cutter.cpp` - Main application file (setup and loop)

## Hardware Details

- ESP32-S3 Development Board
- Stepper Motors (position and cut)
- Multiple sensors and switches
- LEDs for status indication

## Development Environment

- PlatformIO IDE
- Arduino Framework
- Libraries: AccelStepper, Bounce2 