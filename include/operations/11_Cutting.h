#ifndef CUTTING_H
#define CUTTING_H

#include "../core/01_CommonDefinitions.h"
#include "../hardware/05_MotorControl.h"
#include "../hardware/07_SwitchSensor.h"
#include "../hardware/08_PeripheralControl.h"

// Default cutting parameters
#define DEFAULT_CUT_RETRACT_STEPS 5000
#define DEFAULT_CUT_EXTEND_STEPS 5000

// Cut position constants
enum CutPosition {
  CUT_POSITION_SKIP = 0,
  CUT_POSITION_1 = 1,
  CUT_POSITION_2 = 2,
  CUT_POSITION_3 = 3
};

// Function declarations
bool retractCutter(int& nextSubState);
bool extendCutter(int& nextSubState);
void configureForCutPosition(int position, bool& skipCutting);
void signalTransferArm(int state);
void executeCutting();

// Extern variable declarations
extern long cutRetractSteps;
extern long cutExtendSteps;
extern bool hasSuctionBeenChecked;
extern bool hasTransferArmBeenSignaled;
extern bool needCycleSwitchToggle;

#endif 