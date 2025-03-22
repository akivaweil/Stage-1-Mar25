# Stage 1 - CNC Cutting System

This project controls the Stage 1 cutting system using an ESP32 microcontroller. The system manages stepper motors, sensors, and peripheral devices to perform automated wood cutting operations.

## Hardware Requirements

- Freenove ESP32 Breakout Board
- Stepper motors for cutting and positioning
- Limit switches for homing
- Suction system for wood handling
- Transfer arm signaling system
- LED indicators (Red, Green, Blue)

## Pin Layout Reference (ESP32)

**Left side (top to bottom):**
34, 35, 32, 33, 25, 26, 27, 14, 12, 13

**Right side (top to bottom):**
23, 22, 21, 19, 18, 5, 4, 0, 2, 15

## Project Structure

- **src/core/** - Core system files and definitions
- **src/hardware/** - Hardware control modules (motors, LEDs, sensors)
- **src/operations/** - Operation logic and state machine
- **include/** - Header files for all modules

## Key Components

1. **State Machine** - Controls the operational flow through different states
2. **Motor Control** - Manages stepper motors for cutting and positioning
3. **Switch Sensors** - Monitors limit switches and user input switches
4. **LED Control** - Provides visual feedback of system state
5. **Peripheral Control** - Manages external systems like suction and transfer arm

## Operation Flow

1. **Startup** - Initialize systems and perform safety checks
2. **Homing** - Home the motors to their reference positions
3. **Ready** - Wait for user cycle input
4. **Cutting** - Perform the cutting operation
5. **Yes Wood / No Wood** - Handle wood detection outcomes
6. **Error Handling** - Manage error conditions

## WiFi and OTA Configuration

The system supports Over-The-Air (OTA) updates via WiFi:
- SSID: Everwood
- Password: Everwood-Staff
- OTA Password: esp32s3admin

## Setup Instructions

1. Connect the ESP32 to your computer via USB
2. Open the project in PlatformIO
3. Modify any configuration settings as needed in the Config.h file
4. Build and upload the code

## Safety Features

- Startup safety check ensures all systems are in safe positions
- Homing routine with timeouts prevents motor damage
- Error state handling for various fault conditions

## Troubleshooting

- If motors don't home correctly, check limit switch connections
- If suction fails, verify the vacuum system is operational
- For communication issues with the transfer arm, check signal connections

## Recent Improvements

- Added class-based architecture for hardware components
- Centralized configuration parameters in Config.h
- Improved state machine modularity and readability 