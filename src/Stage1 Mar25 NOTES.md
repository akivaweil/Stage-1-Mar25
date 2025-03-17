# Wood Cutting Machine - Stage 1 Functional Notes

## Overview
This automated wood cutting machine utilizes two stepper motors and two pneumatic clamps to precisely cut wood boards to specified lengths. The system implements a specific cutting cycle sequence with integrated safety features, homing capabilities, and error detection systems.

## Hardware Components

### Motors
1. **Cut Motor** - Controls the cutting blade movement across the 8.5-inch travel path
2. **Position Motor** - Controls the positioning of the wood for precise measurement across a 3.45-inch travel path

### Clamps
1. **Position Clamp** - Secures the wood in place during positioning operations and provides additional support during cutting
2. **Secure Wood Clamp** - Firmly holds the wood in position during cutting operations

### Switches and Sensors
1. **Cut Motor Position Switch** - Limit switch that detects when the cut motor is at home position (Active HIGH)
2. **Position Motor Position Switch** - Limit switch that detects when the position motor is at home position (Active HIGH)
3. **Reload Switch** - When activated, enters reload mode to facilitate loading new wood (Active HIGH)
4. **Cycle Switch** - Initiates the cutting cycle when activated (Active HIGH)
5. **Wood Sensor** - Detects the presence of wood (Active LOW - LOW indicates wood is present)
6. **Wood Suction Sensor** - Verifies proper wood suction during cutting operations (Active LOW)

### LED Indicators
1. **Red LED** - Error indicator - blinks during error conditions
2. **Yellow LED** - Operation in progress indicator
3. **Green LED** - Ready indicator - illuminates when system is ready for a cutting cycle and during reload mode
4. **Blue LED** - Setup/special mode indicator - illuminates during startup, when no wood is present, and during reload mode

## Pin Assignments

### ESP32 Breakout Board Pin Layout
- **Left side (top to bottom)**: 34, 35, 32, 33, 25, 26, 27, 14, 12, 13
- **Right side (top to bottom)**: 23, 22, 21, 19, 18, 5, 4, 0, 2, 15

### Motor Control Pins
- Cut Motor Pulse Pin: 22
- Cut Motor Direction Pin: 23
- Position Motor Pulse Pin: 18
- Position Motor Direction Pin: 19

### Switch and Sensor Pins
- Cut Motor Position Switch Pin: 25
- Position Motor Position Switch Pin: 27
- Reload Switch Pin: 26
- Cycle Switch Pin: 14
- Wood Sensor Pin: 35
- Wood Suction Sensor Pin: 5

### Clamp Pins
- Position Clamp Pin: 32
- Wood Secure Clamp Pin: 33

### LED Pins
- Red LED Pin: 26
- Yellow LED Pin: 21
- Green LED Pin: 4
- Blue LED Pin: 2

### Signal Output
- Signal to Stage 2 Pin: 19

## Key Parameters

### Motor Calibration
- Cut Motor: 63.5 steps per inch (200 steps/revolution with 40T pulley on 2GT belt)
- Position Motor: 1000 steps per inch
- Cut Motor Travel Distance: 8.5 inches
- Position Motor Travel Distance: 3.45 inches

### Motor Speeds
- Cut Motor Normal Speed: 80 steps/sec
- Cut Motor Return Speed: 2000 steps/sec
- Position Motor Normal Speed: 40000 steps/sec
- Position Motor Return Speed: 40000 steps/sec

### Motor Acceleration
- Cut Motor Acceleration: 2200 steps/sec²
- Position Motor Acceleration: 40000 steps/sec²
- Position Motor Return Acceleration: 40000 steps/sec²

### Homing Speeds
- Cut Motor Homing Speed: 300 steps/sec
- Position Motor Homing Speed: 2000 steps/sec

### Clamp Operation
- LOW signal = Clamp Extended/Engaged
- HIGH signal = Clamp Retracted

## System States

### 1. STARTUP
- Initial state upon system power-on
- Initializes all pins and system configurations
- Transitions immediately to HOMING state

### 2. HOMING
- Executes the homing sequence to establish reference positions for both motors
- Blue LED blinks at 2-second intervals during homing process
- Transitions to READY state upon completion

### 3. READY
- System awaits user input
- Green LED illuminates to indicate ready status
- Monitors Cycle Switch and Reload Switch for user actions
- Transitions to CUTTING state when cycle is initiated

### 4. CUTTING
- Executes the cutting operation
- Ensures both clamps are properly engaged
- Moves cut motor to the cutting position (8.5 inches)
- Monitors wood suction sensor for proper operation
- Signals Stage 2 when cutting is complete
- Transitions to YESWOOD or NOWOOD state based on wood presence

### 5. YESWOOD
- Handles the sequence when wood is present for further cutting
- Returns both motors to home position
- Moves position motor to final position for next cut
- Manages clamp engagement/retraction during the sequence
- Returns to READY state when complete

### 6. NOWOOD
- Handles the sequence when no wood remains for cutting
- Returns both motors to home position
- Sets the isNoWoodCycleCompleted flag
- Returns to READY state when complete

### 7. ERROR
- Handles various error conditions
- Displays different LED patterns based on error type
- Waits for user acknowledgment via Reload Switch
- Returns to HOMING state when acknowledged

## Cutting Cycle Sequence

The cutting cycle follows these precise steps:

1. **Cycle Start**: Initialize cycle variables and illuminate yellow LED
2. **Clamp Engagement**: Ensure both position and secure wood clamps are extended
3. **Cut Operation**: Cut motor moves to 8.5-inch position
4. **Wood Suction Check**: Verify proper suction during cutting (triggers error if failed)
5. **Signal Stage 2**: Send signal to Stage 2 when cutting is complete
6. **Wood Detection**: Check if wood is present for subsequent operations

For wood present (YESWOOD):
7. **Clamp Management**: Extend position clamp, retract wood secure clamp
8. **Home Return**: Both motors return to home position
9. **Position Clamp Management**: Extend position clamp
10. **Positioning**: Position motor moves 3.45 inches to prepare for next cut
11. **Wood Clamp Engagement**: Extend wood secure clamp
12. **Cycle Complete**: Return to READY state

For no wood present (NOWOOD):
7. **Disengage Clamps**: Retract wood secure clamp
8. **Home Return**: Both motors return to home position
9. **Position Clamp Release**: Retract position clamp
10. **Cycle Complete**: Set no-wood flag and return to READY state

## Homing Sequence

The homing sequence establishes reference positions for both motors:

1. If sensors are already triggered, motors move away by 1 inch first
2. Motors move toward their respective home position switches
3. Motors stop when switches are activated
4. Motors approach switches again at reduced speed for precision
5. Upon final contact, each motor's position is set to zer
6. Position motor moves to operating position (3.45 inches)
7. System is now homed and ready for operation

## Operation Logic

- System initiates by homing both motors upon startup
- In READY state, system monitors for user input via Cycle Switch or Reload Switch
- During reload mode, clamps retract to allow wood loading/unloading
- Cutting cycle executes automatically with appropriate sequence based on wood presence
- Continuous cycling is supported by keeping the Cycle Switch engaged
- Safety check prevents operation if Cycle Switch is ON during startup
- Error detection system monitors wood suction and homing operations

## Safety Features

- Homing sequence ensures accurate reference positions for reliable operation
- Switch debouncing prevents false readings and improper activation
- Clamps are engaged before any cutting operation begins
- Wood suction monitoring detects improper cutting conditions
- Cycle switch safety check prevents unexpected operation at startup
- LED indicators provide clear visual feedback of system status

## Non-Blocking Operation

- System utilizes non-blocking delays via the Wait() function
- Continuous monitoring of switches occurs during delay periods
- Motor operations execute smoothly without blocking the main control loop 