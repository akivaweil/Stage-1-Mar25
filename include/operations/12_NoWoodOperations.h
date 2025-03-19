#ifndef NO_WOOD_OPERATIONS_H
#define NO_WOOD_OPERATIONS_H

#include "../core/01_CommonDefinitions.h"
#include "../hardware/06_LEDControl.h"
#include "../hardware/07_SwitchSensor.h"
#include "../hardware/08_PeripheralControl.h"
#include "09_StateMachine.h"

// Function prototypes for no wood operations
void handleNoWoodState();
void showNoWoodIndicator();

#endif // NO_WOOD_OPERATIONS_H 