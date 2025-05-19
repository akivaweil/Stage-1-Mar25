#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "../core/01_CommonDefinitions.h"
#include "../core/02_PinDefinitions.h"

// LED control functions
void allLedsOff();
void setGreenLed(bool state);
void setBlueLed(bool state);
void setYellowLed(bool state);
void setRedLed(bool state);
void setYesWoodPattern();
void setNoWoodPattern();
void updateBlueBlinkPattern();
void blinkBlueLed();
void setWoodSuctionErrorPattern();
void setCutMotorHomeErrorPattern();
void setPositionMotorHomeErrorPattern();
void setGeneralErrorPattern();
void updateErrorLED(ErrorType errorType);
void updateLEDsForState(State currentState);
void updateAllLEDs(); // Main function called from loop

#endif // LED_CONTROL_H 