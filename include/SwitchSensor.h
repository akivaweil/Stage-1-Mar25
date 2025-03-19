#ifndef SWITCH_SENSOR_H
#define SWITCH_SENSOR_H

#include <Bounce2.h>
#include "CommonDefinitions.h"
#include "PinDefinitions.h"

// Switch debouncing instances
extern Bounce cutMotorHomingSwitch;
extern Bounce positionMotorHomingSwitch;
extern Bounce reloadSwitch;
extern Bounce cycleSwitch;
extern Bounce yesOrNoWoodSensor;
extern Bounce wasWoodSuctionedSensor;

// Constants
extern const unsigned long DEBOUNCE_TIME;

// Boolean flags
extern bool prevCycleSwitchState;
extern bool needCycleSwitchToggle;

// Function prototypes
void initializeDebounce();
void updateAllSwitches();
bool readCutMotorHomingSwitch();
bool readPositionMotorHomingSwitch();
bool readReloadSwitch();
bool readCycleSwitch();
bool isWoodPresent();
bool isWoodSuctionProper();
bool performStartupSafetyCheck();
bool cycleToggleDetected();

#endif // SWITCH_SENSOR_H 