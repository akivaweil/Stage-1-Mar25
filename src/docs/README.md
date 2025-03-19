# Wood Cutting Machine - Stage 1 Documentation

## Overview
This automated wood cutting machine uses two stepper motors and two pneumatic clamps to precisely cut wood boards to specified lengths. The system follows a specific cutting cycle sequence with integrated safety features, homing capabilities, and error detection systems.

## Documentation Sections

### Hardware and System Configuration
- [Hardware Components](01_Hardware_Components.md) - Motors, clamps, switches, and sensors
- [Pin Assignments](02_Pin_Assignments.md) - Pinout and connections
- [Motor Configuration](03_Motor_Configuration.md) - Speeds, acceleration, and calibration

### System Behavior
- [System States](04_System_States.md) - All state definitions and transitions
- [Error Handling](05_Error_Handling.md) - Error detection and recovery procedures
- [Cutting Cycle Sequence](06_Cutting_Cycle.md) - Step-by-step operation
- [Homing Sequence](07_Homing_Sequence.md) - Establishing reference positions
- [Operation Logic](08_Operation_Logic.md) - Overall system logic and behavior

### Implementation Guidelines
- [Code Structure](09_Code_Structure.md) - Function definitions and organization
- [State Machine Implementation](10_State_Machine.md) - State management code templates

### Reference
- [Constants and Variables](12_Constants_Variables.md) - All defined values
- [External Dependencies](13_External_Dependencies.md) - Required libraries

## Getting Started
For new developers, we recommend reading the documentation in the order listed above. The [System States](04_System_States.md) and [Cutting Cycle Sequence](06_Cutting_Cycle.md) documents provide the best overview of how the system works. 