# Stage 1 System Notes

## Overview
This system controls a Stage 1 cutting and positioning machine using an ESP32 microcontroller. The system uses stepper motors for precise movement control, various sensors for position detection, and a state machine architecture for robust operation.

## Hardware Configuration

### ESP32 Breakout Board Pin Layout
- **Left side (top to bottom)**: 34, 35, 32, 33, 25, 26, 27, 14, 12, 13
- **Right side (top to bottom)**: 23, 22, 21, 19, 18, 5, 4, 0, 2, 15

### Pin Assignments
- **Motor Control**
  - Cut Motor: Pulse Pin 22, Direction Pin 23
  - Position Motor: Pulse Pin 32, Direction Pin 33

- **Sensors & Switches**
  - Cut Motor Homing Sensor: Pin 25 (Active HIGH)
  - Position Motor Homing Sensor: Pin 27 (Active HIGH)
  - Reload Switch: Pin 14 (External pull-down)
  - Start Cycle Switch: Pin 18 (External pull-down)
  - Wood Sensor: Pin 35 (Active LOW - LOW means wood present)
  - Wood Suction Sensor: Pin 5 (External pull-down)

- **Clamps**
  - Position Clamp: Pin 13 (LOW to engage)
  - Wood Secure Clamp: Pin 15 (LOW to engage)

- **LEDs**
  - Red LED (Error): Pin 26
  - Yellow LED (Busy/Reload): Pin 21
  - Green LED (Ready): Pin 4
  - Blue LED (Setup/No-Wood): Pin 2

- **Signal Output**
  - Signal to Transfer Arm: Pin 19

## State Machine

The system operates using the following states:

1. **STARTUP**: Initial state when the system powers on
   - Initializes all pins and configurations
   - Transitions immediately to HOMING

2. **HOMING**: Finding home positions for both motors
   - Homes cut motor first, then position motor
   - Moves position motor to starting position (3.45 inches)
   - Blue LED blinks during homing
   - Transitions to READY when complete

3. **READY**: System is ready for operation
   - Green LED on
   - Monitors start switch for cycle initiation
   - Monitors reload switch for reload mode
   - Transitions to CUTTING when start switch is activated

4. **CUTTING**: Performing the cutting operation
   - Ensures clamps are engaged
   - Moves cut motor to cutting position
   - Checks for wood suction error
   - Signals Transfer Arm when complete
   - Transitions to YESwood or NOwood based on wood presence

5. **YESWOOD**: Handling sequence when wood is present
   - Moves position motor back slightly
   - Returns position motor to home while returning cut motor
   - Moves position motor to final position
   - Manages clamps during the entire sequence
   - Returns to READY when complete

6. **NOWOOD**: Handling sequence when no wood is present
   - Returns both motors to home position simultaneously
   - Releases the position clamp
   - Sets the noWoodCycleCompleted flag
   - Returns to READY when complete

7. **ERROR**: Error state
   - Red and yellow LEDs blink
   - Waits for reload switch to acknowledge error
   - Transitions to ERROR_RESET when acknowledged

8. **ERROR_RESET**: Transitioning out of error state
   - Resets error flags and motors
   - Returns to HOMING state

## Key Functions

- **Wait()**: Non-blocking delay function that allows other operations to continue
- **homingSequence()**: Handles the homing process for both motors
- **CUTmovement()**: Controls the cutting operation
- **YESwood()**: Handles the sequence when wood is present (Basically checks whether the board is done)
- **NOwood()**: Handles the sequence when no wood is present 

## Operation Flow

1. System powers on and enters STARTUP state
2. Transitions to HOMING state to find home positions
3. When homing is complete, enters READY state
4. When start switch is activated:
   - System enters CUTTING state
   - Cut motor moves to cutting position
   - System signals Transfer Arm
   - System checks if wood is present
5. If wood is present:
   - System enters YESwood state
   - Position motor moves back slightly
   - Position motor returns to home
   - Position motor moves to final position
6. If no wood is present:
   - System enters NOwood state
   - Both motors return to home
7. System returns to READY state
8. If an error occurs, system enters ERROR state

## Special Features

- **Reload Mode**: Allows for reloading by disengaging clamps
- **Continuous Mode**: Automatically starts new cycles when in continuous mode
- **Start Switch Safety**: Prevents operation if start switch is ON at startup
- **Wood Suction Error Detection**: Detects if wood was suctioned during cutting

## Notes for Maintenance

- Serial communications are commented out for better stepper motor performance
- The system uses non-blocking delays for smooth operation
- Clamps are managed carefully throughout the sequence
- LED indicators show system status at all times 