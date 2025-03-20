#ifndef CUTTING_STATE_HANDLER_H
#define CUTTING_STATE_HANDLER_H

#include "../core/01_CommonDefinitions.h"

// External variables
extern bool hasSuctionBeenChecked;
extern bool hasTransferArmBeenSignaled;
extern bool needCycleSwitchToggle;

// Function prototypes
void handleCuttingState();

#endif // CUTTING_STATE_HANDLER_H 