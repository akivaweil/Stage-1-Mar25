#ifndef PERIPHERAL_CONTROL_H
#define PERIPHERAL_CONTROL_H

#include "CommonDefinitions.h"
#include "PinDefinitions.h"

// Peripheral control functions
void extendPositionClamp();
void retractPositionClamp();
void extendWoodSecureClamp();
void retractWoodSecureClamp();
void signalTransferArm(bool state);
void updateTransferArmSignal();

// Pin initialization
void initializePins();

// External variables used
extern unsigned long transferArmSignalStartTime;
extern bool transferArmSignalActive;

#endif // PERIPHERAL_CONTROL_H 