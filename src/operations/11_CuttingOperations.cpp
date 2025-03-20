#include "../../include/operations/11_CuttingOperations.h"
#include "../../include/core/03_Utilities.h"

// Variables for cutting operations
long cutRetractSteps = DEFAULT_CUT_RETRACT_STEPS;
long cutExtendSteps = DEFAULT_CUT_EXTEND_STEPS;

// Function to retract the cutting mechanism (move up)
bool retractCutter(int& nextSubState) {
  CutMotor_CUTTING_settings();
  
  if (!cutMotor.isRunning()) {
    cutMotor.move(-cutRetractSteps);
  }
  
  if (readCutMotorHomingSwitch() || cutMotor.distanceToGo() == 0) {
    cutMotor.stop();
    nextSubState = 3;  // Go to position adjustment state
    return true;
  }
  
  return false;
}

// Function to extend the cutting mechanism (move down) to cut
bool extendCutter(int& nextSubState) {
  CutMotor_CUTTING_settings();
  
  // If we've hit the emergency limit switch, stop and report error
  if (readCutMotorEmergencySwitch()) {
    cutMotor.stop();
    return true;
  }
  
  if (!cutMotor.isRunning()) {
    cutMotor.move(cutExtendSteps);
  }
  
  if (readCutMotorEmergencySwitch() || cutMotor.distanceToGo() == 0) {
    cutMotor.stop();
    nextSubState = 2;  // Move to retract state
    return true;
  }
  
  return false;
}

// Setup cutting parameters based on what position we need
void configureForCutPosition(int position, bool& skipCutting) {
  switch (position) {
    case CUT_POSITION_1:
      cutRetractSteps = 5000;  // Position 1 retract steps
      cutExtendSteps = 5000;   // Position 1 extend steps
      skipCutting = false;
      break;
      
    case CUT_POSITION_2:
      cutRetractSteps = 6000;  // Position 2 retract steps
      cutExtendSteps = 6000;   // Position 2 extend steps
      skipCutting = false;
      break;
      
    case CUT_POSITION_3:
      cutRetractSteps = 7000;  // Position 3 retract steps
      cutExtendSteps = 7000;   // Position 3 extend steps
      skipCutting = false;
      break;
      
    case CUT_POSITION_SKIP:
      skipCutting = true;
      break;
      
    default:
      cutRetractSteps = DEFAULT_CUT_RETRACT_STEPS;
      cutExtendSteps = DEFAULT_CUT_EXTEND_STEPS;
      skipCutting = false;
      break;
  }
} 