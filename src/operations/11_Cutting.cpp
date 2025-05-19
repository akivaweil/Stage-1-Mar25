#include "../../include/operations/11_Cutting.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/hardware/07_SwitchSensor.h"
#include "../../include/hardware/08_PeripheralControl.h"
#include "../../include/operations/09_StateMachine.h"

// Variables for cutting operations
long cutRetractSteps = DEFAULT_CUT_EXTEND_STEPS;
long cutExtendSteps = DEFAULT_CUT_EXTEND_STEPS;

// Global variables for cutting state
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool needCycleSwitchToggle = false;
static unsigned long transferArmSignalTimer = 0;
static unsigned long woodCheckTimer = 0;

// Function to move the cutting mechanism backward (toward homing switch)
bool retractCutter(int& nextSubState) {
  if (!cutMotor.isRunning()) {
    cutMotor.move(-cutRetractSteps); // Move backward toward homing switch
  }
  
  if (cutMotor.distanceToGo() == 0) {
    nextSubState = nextSubState + 1;
    return true; // Retraction complete
  }
  return false; // Still retracting
}

// Function to move the cutting mechanism forward (away from homing switch) to cut
bool extendCutter(int& nextSubState) {
  static unsigned long startTime = 0;
  
  if (!cutMotor.isRunning()) {
    cutMotor.move(cutExtendSteps); // Move forward away from homing switch
  }
  
  if (cutMotor.distanceToGo() == 0) {
    if (Wait(500, &startTime)) {
      nextSubState = nextSubState + 1;
      return true; // Extension complete with wait
    }
  }
  return false; // Still extending or waiting
}

// Setup cutting parameters based on what position we need
void configureForCutPosition(int position, bool& skipCutting) {
  // Set appropriate cut distance based on position
  switch (position) {
    case 0: // First position
      cutExtendSteps = DEFAULT_CUT_EXTEND_STEPS; // 7.2 inches * steps per inch
      skipCutting = false;
      break;
    default:
      // Default case - use standard settings
      cutExtendSteps = DEFAULT_CUT_EXTEND_STEPS;
      skipCutting = false;
      break;
  }
  
  // Also configure retract steps to match extend steps
  cutRetractSteps = cutExtendSteps;
}

// Signal transfer arm to retrieve the wood piece
void signalTransferArm(int state) {
  digitalWrite(SIGNAL_TO_TRANSFER_ARM_PIN, state);
  hasTransferArmBeenSignaled = (state == HIGH) ? true : hasTransferArmBeenSignaled;
}

// Handle the complete cutting state operation
void executeCutting() {
  static unsigned long startTime = 0;
  
  // Set motor speeds for cutting
  CutMotor_CUTTING_settings();
  
  switch (subState) {
    case 0: // Step 5.1: Initialize cutting cycle
      // Ensure clamps are extended
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // Reset control flags
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      woodCheckTimer = 0;
      
      // Configure for current position
      bool skipCutting;
      configureForCutPosition(0, skipCutting);
      
      // Set cutting indicator LED - Yellow only
      setYellowLed(true);
      setBlueLed(false);
      setGreenLed(false);
      setRedLed(false);
      
      subState = 1;
      break;
      
    case 1: // Step 5.2: Move to cutting position (forward away from home)
      if (extendCutter(subState)) {
        // Extension completed, subState already updated
      }
      break;
      
    case 2: // Step 5.3: Signal transfer arm
      signalTransferArm(HIGH);
      if (Wait(500, &startTime)) {
        signalTransferArm(LOW);
        subState = 3;
      }
      break;
      
    case 3: // Step 5.4: Check if wood is present while cut motor is still at forward position
      // Wait a moment for the wood sensor to stabilize after transfer arm operation
      if (Wait(500, &woodCheckTimer)) {
        // Check wood presence while cut motor is still at forward position (7.2")
        // IMPORTANT: Cut and Position motors are at their extended positions (7.2", 3.45") 
        // when transferring to YESWOOD or NOWOOD states. Those states will handle returning home.
        if (isWoodPresent()) {
          enterState(YESWOOD_STATE);
        } else {
          enterState(NOWOOD_STATE);
        }
      }
      break;
      
    default:
      // Reset to initial substate if we get an invalid state
      subState = 0;
      break;
  }
} 