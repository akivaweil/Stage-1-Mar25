# Wood Cutting Machine - ESP32-S3 Controller Documentation

## Core Design Principles

### 1. Non-Blocking Operations
- All motor movements must be implemented using non-blocking approaches
- All operations must return to the main loop quickly
- The AccelStepper library is used for motor control

### 2. Immediate Command Execution
- No unnecessary delays in command execution
- Commands must execute immediately when issued
- Clamp operations must be instantaneous (no delays)

### 3. Serial Print Limitations
- No Serial.print statements during any motor movements
- Serial printing only allowed in IDLE_STATE, ERROR_STATE, or when motors are confirmed stopped
- Serial prints may be used for diagnostic information when not moving

### 4. Standardized State Machine
- System operates as a finite state machine with clear state transitions
- Each state has well-defined entry and exit conditions
- State machine is implemented in a non-blocking way

### 5. Safety First Design
- Continuous monitoring of safety conditions
- Immediate response to error conditions
- Comprehensive error recovery procedures

## System States

| State | Description | Transition In | Transition Out |
|-------|-------------|---------------|----------------|
| IDLE_STATE | System ready for operation | After homing complete | When start button pressed |
| HOMING_STATE | Finding reference positions | On startup or after error reset | To IDLE_STATE when complete |
| CUTTING_STATE | Performing cutting operation | From IDLE_STATE when cycle starts | To YESWOOD/NOWOOD_STATE when complete |
| ERROR_STATE | Error condition detected | From any state when error detected | To HOMING_STATE after reset |
| YESWOOD_STATE | Wood present detected after cut | From CUTTING_STATE when wood detected | To IDLE_STATE when positioning complete |
| NOWOOD_STATE | No wood detected after cut | From CUTTING_STATE when no wood detected | To IDLE_STATE when complete |

## Documentation Structure

### Core Documentation
1. [**System States**](04_System_States.md) - State machine definitions and transitions
2. [**Operation Logic**](08_Operation_Logic.md) - Overall system behavior and implementation
3. [**Pin Assignments**](02_Pin_Assignments.md) - Hardware connections and logic levels
4. [**Error Handling**](05_Error_Handling.md) - Error detection and recovery procedures

### Operation Documentation
5. [**Homing Sequence**](07_Homing_Sequence.md) - Reference position establishment process
6. [**Cutting Cycle**](06_Cutting_Cycle.md) - Cutting operation workflow
7. [**Motor Configuration**](03_Motor_Configuration.md) - Motor parameters and settings

### Additional Reference
8. [**Hardware Components**](01_Hardware_Components.md) - Physical components used in the system
9. [**Code Structure**](09_Code_Structure.md) - Software architecture guidelines
10. [**Constants and Variables**](12_Constants_Variables.md) - System constants reference

## Getting Started

### 1. System Overview
The wood cutting machine uses two stepper motors to control a cutting mechanism and a positioning system. The machine cuts wooden boards to precise lengths through a controlled sequence of operations. The system uses position switches for homing, sensors for detecting the presence of wood, and pneumatic clamps for securing the wood during cutting.

### 2. Key Components
- **ESP32-S3 Controller**: Main control board (Freenove ESP32-S3)
- **Stepper Motors**: For cut and position control
- **Pneumatic Clamps**: For securing wood during operations
- **Position Switches**: For detecting home positions (active HIGH)
- **Sensors**: For detecting wood presence and suction (active LOW)

### 3. Implementation Requirements
- Use non-blocking programming techniques
- Implement proper debouncing (15ms) for all inputs
- Verify all logic levels match documentation (active HIGH/LOW)
- Ensure clamp operations occur immediately
- Maintain safety checks at all times

### 4. Getting Started for Developers
1. Read the [System States](04_System_States.md) document to understand the state machine
2. Review the [Pin Assignments](02_Pin_Assignments.md) to understand hardware connections
3. Study the [Homing Sequence](07_Homing_Sequence.md) and [Cutting Cycle](06_Cutting_Cycle.md) documents
4. Implement the core state machine as described in [Operation Logic](08_Operation_Logic.md)

## Logic Levels Reference

| Input/Output | Logic Level | Active State |
|--------------|-------------|--------------|
| Position Switches | Active HIGH | HIGH when triggered |
| Button Inputs | Active LOW | LOW when pressed |
| Wood Present Sensor | Active LOW | LOW when wood present |
| Vacuum Sensor | Active LOW | HIGH when suction proper |
| Clamp Outputs | Active LOW | LOW to extend (engage) |
| LED Outputs | Active HIGH | HIGH to illuminate |
| Transfer Arm Signal | Active HIGH | HIGH pulse for 500ms |

## E-Stop Implementation
The Emergency Stop button is a hardware cut-off that directly interrupts power to the motors and does not connect to the ESP32-S3. This physical safety mechanism operates independently of the controller software. 