#include "../../include/core/00_WifiAndOTA.h"

// WiFi credentials
const char* ssid = "Everwood";         // WiFi network name
const char* password = "Everwood-Staff"; // WiFi password

// OTA Password - change this for security
const char* otaPassword = "esp32s3admin";

// Variables for connection status
bool wifiConnected = false;
unsigned long connectionStartTime = 0;
const unsigned long connectionTimeout = 15000; // 15 seconds timeout
unsigned long lastStatusPrintTime = 0;

// Sets up OTA update functionality
void setupOTA() {
  // Hostname defaults to esp32-[MAC/CPU ID]
  ArduinoOTA.setHostname("esp32-s3-ota");

  // Set authentication password
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

// Attempt to connect to WiFi
void connectToWiFi() {
  if (wifiConnected) return;
  
  Serial.println("Connecting to WiFi network: " + String(ssid));
  
  // Connect to WiFi network
  WiFi.disconnect(true);  // Disconnect from any previous connections
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Set connection start time for timeout
  connectionStartTime = millis();
  
  // Wait for connection with short timeout
  // We use a non-blocking approach here
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println(">>> SUCCESSFULLY CONNECTED TO WIFI <<<");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Setup OTA updates
    setupOTA();
  }
}

// Disconnect from WiFi
void disconnectWiFi() {
  if (!wifiConnected) return;
  
  Serial.println("Disconnecting from WiFi...");
  WiFi.disconnect(true);
  wifiConnected = false;
}

// Initialize WiFi settings but don't connect yet
void initializeWiFi() {
  // Print MAC address
  Serial.print("Device MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // WiFi will be connected only when in READY_STATE
  wifiConnected = false;
}

// Update WiFi and OTA based on machine state
void updateWiFiAndOTA() {
  // Only handle WiFi and OTA in READY_STATE
  if (currentState == READY_STATE) {
    // Try to connect to WiFi if not connected
    if (!wifiConnected && WiFi.status() != WL_CONNECTED) {
      // Attempt connection if enough time has passed since last attempt
      static unsigned long lastConnectionAttempt = 0;
      if (millis() - lastConnectionAttempt > 30000) { // Try every 30 seconds
        connectToWiFi();
        lastConnectionAttempt = millis();
      }
    } 
    // Handle reconnection if lost
    else if (wifiConnected && WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost! Attempting to reconnect...");
      wifiConnected = false;
    }
    // Handle successful reconnection
    else if (!wifiConnected && WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println("WiFi reconnected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      
      // Setup OTA updates
      setupOTA();
    }
    
    // Handle OTA updates only when connected
    if (wifiConnected) {
      ArduinoOTA.handle();
    }
    
    // Periodically print status (every 30 seconds)
    if (wifiConnected && millis() - lastStatusPrintTime > 30000) {
      Serial.println("\n--- WIFI STATUS ---");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Signal Strength: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      Serial.println("-------------------");
      
      lastStatusPrintTime = millis();
    }
  } 
  // Disconnect from WiFi if not in READY_STATE
  else if (wifiConnected) {
    disconnectWiFi();
  }
} 