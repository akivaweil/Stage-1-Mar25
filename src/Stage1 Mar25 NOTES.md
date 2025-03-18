# Wood Cutting Machine - Stage 1 Functional Notes

## Overview
This automated wood cutting machine uses two stepper motors and two pneumatic clamps to precisely cut wood boards to specified lengths. The system follows a specific cutting cycle sequence with integrated safety features, homing capabilities, and error detection systems.

## Hardware Components

### Motors
1. **Cut Motor** - Controls the cutting blade movement across the 7.7-inch travel path
2. **Position Motor** - Controls the wood positioning for precise movement across a 3.45-inch travel path

### Clamps
1. **Position Clamp** - Secures the wood during positioning operations and provides additional support during cutting
2. **Secure Wood Clamp** - Firmly holds the wood in position during cutting operations

### Switches and Sensors
1. **Cut Motor Position Switch** - Limit switch that detects when the cut motor is at home position (HIGH when triggered)
2. **Position Motor Position Switch** - Limit switch that detects when the position motor is at home position (HIGH when triggered)
3. **Reload Switch** - Activates reload mode for loading new wood (HIGH when triggered)
4. **Cycle Switch** - Initiates the cutting cycle (HIGH when triggered)
5. **Wood Sensor** - Detects the presence of wood (LOW when wood is present)
6. **Wood Suction Sensor** - Verifies proper wood suction during cutting operations (LOW when suction is correct)

### LED Indicators
1. **Red LED** - Error indicator - blinks with pattern indicating specific error type
2. **Yellow LED** - Operation in progress indicator
3. **Green LED** - Ready indicator - illuminates when system is ready for cutting cycle and during reload mode
4. **Blue LED** - Setup/special mode indicator - illuminates during startup, when no wood is present, and during reload mode

## Pin Assignments

### ESP32-S3 Breakout Board Pin Layout
- **Left side (top to bottom)**: 4, 5, 6, 7, 15, 16, 17, 18, 8, 3, 46, 9, 10, 11, 12, 13, 14
- **Right side (top to bottom)**: 1, 2, 42, 41, 40, 39, 38, 37, 36, 35, 0, 45, 48, 47, 21, 20, 19

### Motor Control Pins
- Cut Motor Pulse Pin: 39
- Cut Motor Direction Pin: 38
- Position Motor Pulse Pin: 1
- Position Motor Direction Pin: 2

### Switch and Sensor Pins
- Cut Motor Position Switch Pin: 10
- Position Motor Position Switch Pin: 9
- Reload Switch Pin: 14
- Cycle Switch Pin: 13
- Wood Sensor Pin: 11
- Wood Suction Sensor Pin: 12

### Clamp Pins
- Position Clamp Pin: 18
- Wood Secure Clamp Pin: 17

### LED Pins
- Red LED Pin: 16
- Yellow LED Pin: 15
- Green LED Pin: 7
- Blue LED Pin: 6

### Signal Output
- Signal to Transfer Arm (TA) Pin: 19

## Key Parameters

### Motor Calibration
- Cut Motor: 76 steps per inch
- Position Motor: 1000 steps per inch
- Cut Motor Travel Distance: 7.7 inches
- Position Motor Travel Distance: 3.45 inches

### Motor Speeds
- Cut Motor Normal Speed: 105 steps/sec
- Cut Motor Return Speed: 1500 steps/sec
- Position Motor Normal Speed: 30000 steps/sec
- Position Motor Return Speed: 30000 steps/sec

### Motor Acceleration
- Cut Motor Acceleration: 2500 steps/sec²
- Position Motor Acceleration: 30000 steps/sec²
- Position Motor Return Acceleration: 30000 steps/sec²

### Homing Speeds
- Cut Motor Homing Speed: 300 steps/sec
- Position Motor Homing Speed: 2000 steps/sec

### Clamp Operation
- LOW signal = Clamp Extended/Engaged
- HIGH signal = Clamp Retracted

## System States

### 1. STARTUP
- Initial state when system powers on
- Initializes all pins and system configurations
- Transitions directly to HOMING state

### 2. HOMING
- Runs the homing sequence to establish reference positions for both motors
- Blue LED blinks at 2-second intervals during this process
- Transitions to READY state when complete

### 3. READY
- System waits for user input
- Green LED illuminates to indicate ready status
- Monitors Cycle Switch and Reload Switch for user actions
- Moves to RELOAD state when Reload Switch is activated
- Moves to CUTTING state when Cycle Switch is activated

### 4. RELOAD
- Accessible only from the READY state
- Retracts both clamps to allow wood loading/unloading
- Yellow LED and Blue LED illuminate to indicate reload mode
- Monitors Reload Switch for deactivation
- Must return to READY state when Reload Switch is released before any cutting operation can begin
- Cannot directly transition to CUTTING state as a safety measure

### 5. CUTTING
- Performs the cutting operation
- Verifies both clamps are properly engaged
- Moves cut motor to the cutting position (7.7 inches)
- Checks wood suction sensor at 0.5 inch into the cut
- Signals the Transfer Arm (TA) when cut motor reaches 7.2 inches (0.5 inches before end of travel)
- Transitions to YESWOOD or NOWOOD state based on wood sensor reading

### 6. YESWOOD
- Handles sequence when wood is present for further cutting
- Returns both motors to home position
- Moves position motor to final position for next cut
- Manages clamp engagement/retraction during the sequence
- Returns to READY state when complete

### 7. NOWOOD
- Handles sequence when no wood remains for cutting
- Returns both motors to home position
- Sets the isNoWoodCycleCompleted flag
- Returns to READY state when complete

### 8. ERROR
- Manages various error conditions
- Displays distinct LED patterns based on error type
- Waits for user acknowledgment via Reload Switch
- Returns to HOMING state when acknowledged

## Error States

The system enters the ERROR state due to specific detected conditions:

### 1. WOOD_SUCTION_ERROR
- Triggered when: Wood suction sensor indicates failure (Active LOW) at the 0.5-inch point in the cut
- Visual indicator: Red LED blinks in pattern of 3 quick flashes followed by a pause
- Resolution: Operator must check vacuum system and remove any debris or blockage
- Recovery: System returns to HOMING state, then requires operator to turn OFF and ON the Cycle Switch to resume operation

### 2. CUT_MOTOR_HOME_ERROR
- Triggered when: Cut motor position switch does not read HIGH after motor completes return to home position
- Visual indicator: Red LED blinks in pattern of 2 quick flashes followed by a pause
- Possible causes: Mechanical obstruction, motor step loss, or switch failure
- Recovery: Cut motor moves 1 inch forward then attempts to rehome. If rehoming fails again, the motor moves forward 1 inch once more and enters the HOMING state

## Cutting Cycle Sequence

The cutting cycle follows these specific steps:

1. **Cycle Start**: Initialize cycle variables and turn on yellow LED
2. **Clamp Verification**: Confirm both position and secure wood clamps are extended
3. **Cut Operation**: Move cut motor to 7.7-inch position
4. **Wood Suction Check**: Check wood suction sensor exactly when the cut motor reaches 0.5 inch into the cut. If sensor indicates a problem (is HIGH), immediately trigger an error condition and halt the cutting operation.
5. **Signal Transfer Arm**: Send signal to the Transfer Arm (TA) when the cut motor reaches 7.2 inches (0.5 inches before end of travel)
6. **Wood Detection**: Check if wood is present for subsequent operations

For wood present (YESWOOD):
7. **Initial Return**: Start both motors moving to their home positions
8. **Initial Clamp Management**: When the position motor has moved 0.1 inches toward home, retract the position clamp while keeping wood secure clamp extended
9. **Complete Return**: Continue movement until both motors reach their home position
10. **Home Position Check**: Wait 50ms after the cut motor reaches position zero, then verify the cut motor position switch reads HIGH. If not, trigger the CUT_MOTOR_HOME_ERROR state
11. **Position Clamp Engagement**: Extend position clamp once position zero is reached
12. **Positioning Operation**: Move position motor to 3.45 inches for next cut
13. **Wood Clamp Verification**: Confirm wood secure clamp remains extended
14. **Cycle Complete**: Return to READY state

For no wood present (NOWOOD):
7. **Clamp Management**: Retract wood secure clamp
8. **Home Return**: Both motors return to home position
9. **Position Clamp Release**: Retract position clamp
10. **Cycle Complete**: Set no-wood flag and return to READY state

## Homing Sequence

The homing sequence establishes reference positions for both motors:

1. If position switches are already triggered, motors move away by 1 inch first
2. Motors move toward their respective home position switches at homing speed
3. Motors stop immediately when switches are activated and positions are set to 0
4. Position motor moves to operating position (3.45 inches)
5. System is now homed and ready for operation

## Operation Logic

- System performs homing sequence immediately after startup
- In READY state, system monitors Cycle Switch and Reload Switch inputs
- RELOAD state is only accessible from READY state by activating the Reload Switch
- Both clamps remain retracted during RELOAD state to facilitate wood handling
- System must return to READY state from RELOAD before starting a cutting cycle
- This safety feature prevents accidental cycle activation during material loading
- Wood suction is checked exactly once at 0.5 inch into the cut
- Transfer Arm signal is sent precisely at 7.2 inches of cut motor travel
- Machine continues cycling if Cycle Switch remains activated until NOWOOD state is completed
- Safety check prevents operation if Cycle Switch is ON during startup
- Error detection continuously monitors for wood suction and homing failures

## Safety Features

- Homing sequence ensures accurate reference positions for reliable operation
- Switch debouncing prevents false readings and improper activations
- Both clamps must be engaged before cutting begins
- Wood suction monitoring detects improper cutting conditions early in the process
- Cycle Switch safety check prevents unexpected operation at startup
- RELOAD state requires returning to READY state before cutting operations can begin
- Clear LED indicators provide visual feedback of system status

## Non-Blocking Operation

- System uses non-blocking delays via the Wait() function
- Continuous monitoring of switches occurs during all delay periods
- Motor operations execute without blocking the main control loop 

## Code Implementation Requirements

### Libraries
- **AccelStepper Library** - Required for all motor control operations
- **Bounce2 Library** - Required for debouncing all switches and sensors

### Debounce Configuration
- All switches and sensors must use a 20ms debounce time
- This includes:
  - Cut Motor Position Switch
  - Position Motor Position Switch
  - Reload Switch
  - Cycle Switch
  - Wood Sensor
  - Wood Suction Sensor 