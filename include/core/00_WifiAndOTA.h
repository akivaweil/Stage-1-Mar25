#ifndef WIFI_AND_OTA_H
#define WIFI_AND_OTA_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "01_CommonDefinitions.h"

// External function declarations
void initializeWiFi();
void updateWiFiAndOTA();
void disconnectWiFi();
void setupOTA();

// External variable declarations
extern bool wifiConnected;

#endif // WIFI_AND_OTA_H 