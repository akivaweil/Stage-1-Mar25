#include "../../include/operations/11_Cutting.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/hardware/07_SwitchSensor.h"
#include "../../include/hardware/08_PeripheralControl.h"
#include "../../include/operations/09_StateMachine.h"

// Variables for cutting operations
long cutRetractSteps = DEFAULT_CUT_RETRACT_STEPS;
long cutExtendSteps = DEFAULT_CUT_EXTEND_STEPS;

// Global variables for cutting state
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool needCycleSwitchToggle = false;
static unsigned long transferArmSignalTimer = 0;

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
      cutExtendSteps = 5000;
      cutRetractSteps = 5000;
      skipCutting = false;
      break;
    
    case 1: // Second position
      cutExtendSteps = 4800;
      cutRetractSteps = 4800;
      skipCutting = false;
      break;
      
    // Add more positions as needed
    
    default:
      // Default values
      cutExtendSteps = DEFAULT_CUT_EXTEND_STEPS;
      cutRetractSteps = DEFAULT_CUT_RETRACT_STEPS;
      skipCutting = false;
      break;
  }
}

// Function to signal the transfer arm
void signalTransferArm() {
  static unsigned long startTime = 0;
  
  if (!hasTransferArmBeenSignaled) {
    signalTransferArm(HIGH);
    hasTransferArmBeenSignaled = true;
    startTime = millis();
  }
  
  // Auto turn off after 500ms
  if (hasTransferArmBeenSignaled && (millis() - startTime >= 500)) {
    signalTransferArm(LOW);
  }
}

// Handle the complete cutting state operation
void executeCutting() {
  static unsigned long startTime = 0;
  
  // Set motor speeds for cutting
  CutMotor_CUTTING_settings();
  
  switch (subState) {
    case 0: // Initialize cutting cycle
      // Ensure clamps are extended
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // Reset control flags
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      
      // Configure for current position
      bool skipCutting;
      configureForCutPosition(0, skipCutting);
      
      subState = 1;
      break;
      
    case 1: // Move to cutting position (forward away from home)
      if (extendCutter(subState)) {
        // Extension completed, subState already updated
      }
      break;
      
    case 2: // Signal transfer arm
      signalTransferArm(HIGH);
      if (Wait(500, &startTime)) {
        signalTransferArm(LOW);
        subState = 3;
      }
      break;
      
    case 3: // Return cut motor home (backward toward homing switch)
      if (retractCutter(subState)) {
        // Retraction completed, subState already updated
      }
      break;
      
    case 4: // Check if wood is present
      if (isWoodPresent()) {
        enterState(YESWOOD_STATE);
      } else {
        enterState(NOWOOD_STATE);
      }
      break;
  }
} 