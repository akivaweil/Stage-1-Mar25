#ifndef SWITCH_SENSOR_H
#define SWITCH_SENSOR_H

#include "CommonDefinitions.h"
#include "PinDefinitions.h"
#include <Bounce2.h>

// Switch and sensor declarations
extern Bounce cutMotorHomingSwitch;
extern Bounce positionMotorHomingSwitch;
extern Bounce reloadSwitch;
extern Bounce cycleSwitch;
extern Bounce yesOrNoWoodSensor;
extern Bounce wasWoodSuctionedSensor;
extern bool prevCycleSwitchState;
extern bool needCycleSwitchToggle;
extern unsigned long DEBOUNCE_TIME;

// Switch and sensor functions
void initializeDebounce();
void updateAllSwitches();
bool readCutMotorHomingSwitch();
bool readPositionMotorHomingSwitch();
bool readReloadSwitch();
bool readCycleSwitch();
bool isWoodPresent();
bool isWoodSuctionProper();
bool cycleToggleDetected();
bool performStartupSafetyCheck();

#endif // SWITCH_SENSOR_H 