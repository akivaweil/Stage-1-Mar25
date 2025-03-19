# Cutting Cycle

This document describes the cutting cycle for the wood cutting machine.

## Cutting Cycle Overview

The cutting cycle consists of the following sequential steps:

1. Check that system is in READY_STATE and properly homed
2. Detect cycle start trigger (cycle switch active HIGH)
3. Extend position clamp to secure wood (immediate operation)
4. Move cut motor through the cutting sequence while monitoring sensors
5. Signal transfer arm at appropriate position
6. Complete cut and check if wood is present
7. Based on wood presence, transition to either YESWOOD_STATE or NOWOOD_STATE

## Non-Blocking Implementation

The cutting cycle must be implemented as a non-blocking state machine:

```cpp
// Cutting state is managed using the subState variable

void handleCuttingState() {
  // Implementation for the cutting state
  // This is a state machine within a state machine
  
  switch (subState) {
    case 0:  // Init cutting cycle
      // Initialize by ensuring clamps are extended
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // Reset control flags
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      
      // Proceed to next substate
      subState = 1;
      break;
      
    case 1:  // Move to suction check position
      // Set motor speed
      cutMotor.setMaxSpeed(CUT_MOTOR_NORMAL_SPEED);
      cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
      
      // Start move to suction check position if not already moving
      if (!cutMotor.isRunning()) {
        cutMotor.moveTo(inchesToSteps(WOOD_SUCTION_CHECK_POSITION, CUT_MOTOR_STEPS_PER_INCH));
      }
      
      // Check if position reached
      if (isMotorInPosition(cutMotor, inchesToSteps(WOOD_SUCTION_CHECK_POSITION, CUT_MOTOR_STEPS_PER_INCH))) {
        subState = 2;  // Proceed to suction check
      }
      break;
      
    case 2:  // Check suction
      // Check if wood is properly suctioned (sensor reads HIGH when proper)
      if (isWoodSuctionProper()) {
        // Wood suction is proper, continue to next step
        hasSuctionBeenChecked = true;
        subState = 3;  // Continue cutting
      } else {
        // Wood suction failed, go to error state
        enterState(WOOD_SUCTION_ERROR_STATE);
        return;
      }
      break;
      
    case 3:  // Continue cutting to transfer arm signal position
      // Start move to transfer arm signal position
      cutMotor.moveTo(inchesToSteps(TRANSFER_ARM_SIGNAL_POSITION, CUT_MOTOR_STEPS_PER_INCH));
      
      // Check if position reached
      if (isMotorInPosition(cutMotor, inchesToSteps(TRANSFER_ARM_SIGNAL_POSITION, CUT_MOTOR_STEPS_PER_INCH))) {
        subState = 4;  // Proceed to signal transfer arm
      }
      break;
      
    case 4:  // Signal transfer arm
      // Signal transfer arm if not already done
      if (!hasTransferArmBeenSignaled) {
        signalTransferArm(HIGH);
        hasTransferArmBeenSignaled = true;
      }
      
      // Allow 500ms for the signal (handled by signalTransferArm function)
      // Then continue cutting - non-blocking handled by updateTransferArmSignal in main loop
      subState = 5;
      break;
      
    case 5:  // Complete cut
      // Move to complete cut position
      cutMotor.moveTo(inchesToSteps(CUT_MOTOR_TRAVEL_DISTANCE, CUT_MOTOR_STEPS_PER_INCH));
      
      // Check if cut completed
      if (isMotorInPosition(cutMotor, inchesToSteps(CUT_MOTOR_TRAVEL_DISTANCE, CUT_MOTOR_STEPS_PER_INCH))) {
        subState = 6;  // Proceed to check for wood presence
      }
      break;
      
    case 6:  // Check wood presence
      // Check if wood is present at the end of cut cycle
      if (isWoodPresent()) {
        // Wood detected - transition to yeswood state
        enterState(YESWOOD_STATE);
      } else {
        // No wood detected - transition to nowood state
        enterState(NOWOOD_STATE);
      }
      break;
  }
}
```

## Wood Present State (YESWOOD_STATE)

The YESWOOD_STATE handles the case when wood is detected after cutting:

```cpp
void handleYesWoodState() {
  switch (subState) {
    case 0:  // Prepare for next position
      // Keep wood secure clamp extended, retract position clamp
      retractPositionClamp();
      
      // Set position motor speed
      positionMotor.setMaxSpeed(POSITION_MOTOR_NORMAL_SPEED);
      positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
      
      subState = 1;
      break;
      
    case 1:  // Move position motor for next cut
      // Start movement if not already moving
      if (!positionMotor.isRunning()) {
        positionMotor.moveTo(inchesToSteps(POSITION_MOTOR_TRAVEL_DISTANCE, POSITION_MOTOR_STEPS_PER_INCH));
      }
      
      // Check if position reached
      if (isMotorInPosition(positionMotor, inchesToSteps(POSITION_MOTOR_TRAVEL_DISTANCE, POSITION_MOTOR_STEPS_PER_INCH))) {
        subState = 2;
      }
      break;
      
    case 2:  // Return cut motor to home
      // Set cut motor speed for return
      cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
      cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
      
      // Start movement if not already moving
      if (!cutMotor.isRunning()) {
        cutMotor.moveTo(0);
      }
      
      // Check if home reached
      if (isMotorInPosition(cutMotor, 0)) {
        // Return to ready state for next cycle
        enterState(READY_STATE);
      }
      break;
  }
}
```

## No Wood Present State (NOWOOD_STATE)

The NOWOOD_STATE handles the case when no wood is detected after cutting:

```cpp
void handleNoWoodState() {
  switch (subState) {
    case 0:  // Prepare for return to home
      // Retract both clamps
      retractPositionClamp();
      retractWoodSecureClamp();
      
      // Set cut motor speed for return
      cutMotor.setMaxSpeed(CUT_MOTOR_RETURN_SPEED);
      cutMotor.setAcceleration(CUT_MOTOR_ACCELERATION);
      
      subState = 1;
      break;
      
    case 1:  // Return cut motor to home
      // Start movement if not already moving
      if (!cutMotor.isRunning()) {
        cutMotor.moveTo(0);
      }
      
      // Check if home reached
      if (isMotorInPosition(cutMotor, 0)) {
        subState = 2;
      }
      break;
      
    case 2:  // Return position motor to home
      // Set position motor speed for return
      positionMotor.setMaxSpeed(POSITION_MOTOR_RETURN_SPEED);
      positionMotor.setAcceleration(POSITION_MOTOR_ACCELERATION);
      
      // Start movement if not already moving
      if (!positionMotor.isRunning()) {
        positionMotor.moveTo(0);
      }
      
      // Check if home reached
      if (isMotorInPosition(positionMotor, 0)) {
        // Return to ready state for next cycle
        enterState(READY_STATE);
      }
      break;
  }
}
```

## Transfer Arm Signal

The transfer arm is signaled with a 500ms HIGH pulse at a specific position during the cutting cycle:

```cpp
// Function to signal the transfer arm (non-blocking 500ms HIGH pulse)
void signalTransferArm(bool state) {
  if (state == HIGH && !transferArmSignalActive) {
    // Turn on the signal and mark start time
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, HIGH);
    transferArmSignalStartTime = millis();
    transferArmSignalActive = true;
  } else if (state == LOW) {
    // Immediately turn off the signal
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
    transferArmSignalActive = false;
  }
}

// Function to update transfer arm signal timing (call this in loop)
void updateTransferArmSignal() {
  if (transferArmSignalActive && (millis() - transferArmSignalStartTime >= 500)) {
    // 500ms has elapsed, turn off the signal
    digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, LOW);
    transferArmSignalActive = false;
  }
}
```

## Wood Suction Error Handling

If wood suction fails during the cutting cycle, the system transitions to WOOD_SUCTION_ERROR_STATE:

```cpp
void handleWoodSuctionErrorState() {
  // Keep updating the error LED pattern
  updateRedLEDErrorPattern(WOOD_SUCTION_ERROR);
  
  // Keep clamps extended for safety
  extendPositionClamp();
  extendWoodSecureClamp();
  
  // Return motors to home position if they're not already there
  if (!isMotorInPosition(cutMotor, 0)) {
    moveCutMotorToPosition(0);
  }
  
  if (!isMotorInPosition(positionMotor, 0)) {
    movePositionMotorToPosition(0);
  }
  
  // Monitor cycle switch for toggle (OFF then ON)
  bool currentCycleSwitchState = readCycleSwitch();
  
  // Check for a complete toggle cycle (OFF then ON)
  if (prevCycleSwitchState == false && currentCycleSwitchState == true) {
    // Cycle switch has been toggled off and then on again
    // Reset error and go to homing state
    currentError = NO_ERROR;
    enterState(HOMING_STATE);
  }
  
  // Update the previous cycle switch state
  prevCycleSwitchState = currentCycleSwitchState;
}
```

## Position Check Helper Functions

Helper functions are used to check motor positions and sensor readings:

```cpp
// Check if a motor is at the specified position (within small tolerance)
bool isMotorInPosition(AccelStepper& motor, long position) {
  return (abs(motor.currentPosition() - position) < 5) && !motor.isRunning();
}

// Move cut motor to a specific position (in steps)
void moveCutMotorToPosition(float inches) {
  long steps = inchesToSteps(inches, CUT_MOTOR_STEPS_PER_INCH);
  cutMotor.moveTo(steps);
}

// Move position motor to a specific position (in steps)
void movePositionMotorToPosition(float inches) {
  long steps = inchesToSteps(inches, POSITION_MOTOR_STEPS_PER_INCH);
  positionMotor.moveTo(steps);
}
```

## Reload State

The RELOAD_STATE is used when the reload switch is activated to allow wood loading:

```cpp
void handleReloadState() {
  // Set blue LED to indicate reload mode
  digitalWrite(BLUE_LED_PIN, HIGH);
  
  // Retract both clamps for reloading
  retractPositionClamp();
  retractWoodSecureClamp();
  
  // Check if reload switch is deactivated
  if (!readReloadSwitch()) {
    // Return to ready state when reload switch is deactivated
    enterState(READY_STATE);
  }
}
```

## State Transitions

The cutting cycle involves these key state transitions:

1. READY_STATE → CUTTING_STATE (when cycle switch activated)
2. CUTTING_STATE → YESWOOD_STATE (when wood detected after cut)
3. CUTTING_STATE → NOWOOD_STATE (when no wood detected after cut)
4. CUTTING_STATE → WOOD_SUCTION_ERROR_STATE (when suction fails)
5. YESWOOD_STATE → READY_STATE (after positioning for next cut)
6. NOWOOD_STATE → READY_STATE (after returning to home position)

Each transition occurs based on specific conditions with no unnecessary delays. 