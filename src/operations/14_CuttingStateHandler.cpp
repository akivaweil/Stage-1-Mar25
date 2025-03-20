#include "../../include/operations/09_StateMachine.h"
#include "../../include/core/03_Utilities.h"
#include "../../include/hardware/07_SwitchSensor.h"
#include "../../include/hardware/08_PeripheralControl.h"

// Global variables for cutting state
bool hasSuctionBeenChecked = false;
bool hasTransferArmBeenSignaled = false;
bool needCycleSwitchToggle = false;
static unsigned long transferArmSignalTimer = 0;

// Handle cutting state
void handleCuttingState() {
  static unsigned long startTime = 0;
  
  // Set motor speeds
  CutMotor_CUTTING_settings();
  
  switch (subState) {
    case 0: // Initialize cutting cycle
      // Ensure clamps are extended
      extendPositionClamp();
      extendWoodSecureClamp();
      
      // Reset control flags
      hasSuctionBeenChecked = false;
      hasTransferArmBeenSignaled = false;
      
      subState = 1;
      break;
      
    case 1: // Move to cutting position
      if (!cutMotor.isRunning()) {
        cutMotor.move(5000); // Move down to cutting position
      }
      
      if (cutMotor.distanceToGo() == 0) {
        if (Wait(500, &startTime)) {
          subState = 2;
        }
      }
      break;
      
    case 2: // Signal transfer arm
      signalTransferArm(HIGH);
      if (Wait(500, &startTime)) {
        signalTransferArm(LOW);
        subState = 3;
      }
      break;
      
    case 3: // Return cut motor home
      if (!cutMotor.isRunning()) {
        cutMotor.move(-5000); // Move back up
      }
      
      if (cutMotor.distanceToGo() == 0) {
        subState = 4;
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