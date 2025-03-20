#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "CommonDefinitions.h"
#include "PinDefinitions.h"

// LED control functions
void allLedsOff();
void setGreenLed(bool state);
void setBlueLed(bool state);
void setYellowLed(bool state);
void setRedLed(bool state);
void setYesWoodPattern();
void setNoWoodPattern();
void updateBlueBlinkPattern();
void setWoodSuctionErrorPattern();
void setCutMotorHomeErrorPattern();
void setPositionMotorHomeErrorPattern();
void setGeneralErrorPattern();
void updateRedLEDErrorPattern(ErrorType errorType);
void updateLEDsForState(State currentState);

#endif // LED_CONTROL_H 