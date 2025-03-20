/*
  ESP32-S3 IP Address Finder
  
  This sketch connects the ESP32-S3 to WiFi and displays the IP address
  over Serial. This allows for future OTA updates by knowing the device's IP.
  
  Freenove ESP32-S3 Breakout Board
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// WiFi credentials
const char* ssid = "Everwood";         // WiFi network name
const char* password = "Everwood-Staff"; // WiFi password

// OTA Password - change this for security
const char* otaPassword = "esp32s3admin";

// Variables for connection status
bool wifiConnected = false;
unsigned long connectionStartTime = 0;
const unsigned long connectionTimeout = 15000; // 15 seconds timeout

// Non-blocking delay function
bool Wait(unsigned long delayTime, unsigned long* startTimePtr) {
  // First time entering this function
  if (*startTimePtr == 0) {
    *startTimePtr = millis();
    return false;
  }
  
  // Check if the delay time has elapsed
  if (millis() - *startTimePtr >= delayTime) {
    *startTimePtr = 0;  // Reset for next use
    return true;
  }
  
  return false;
}

void setupOTA() {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

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
    
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
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

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(1000); // Brief delay to allow Serial to initialize
  
  Serial.println("\n\n--- ESP32-S3 IP Address Finder ---");
  Serial.println("Connecting to WiFi...");
  
  // Connect to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Set connection start time for timeout
  connectionStartTime = millis();
  
  // Wait for connection with timeout
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - connectionStartTime > connectionTimeout) {
      Serial.println("WiFi connection failed! Please check your credentials or network availability.");
      // Go into an infinite loop or restart
      while (true) {
        delay(1000);
      }
    }
    delay(500);
    Serial.print(".");
  }
  
  wifiConnected = true;
  Serial.println("");
  Serial.print("Connected to WiFi network: ");
  Serial.println(ssid);
  
  // Print the IP address
  Serial.println("\n--- DEVICE INFORMATION ---");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("RSSI (Signal Strength): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.println("---------------------------");
  
  // Setup OTA updates
  setupOTA();
  
  Serial.println("\nTo enable OTA uploads:");
  Serial.println("1. In platformio.ini, uncomment the OTA upload settings");
  Serial.println("2. Set upload_port to: " + WiFi.localIP().toString());
  Serial.println("3. Set the OTA password in both the sketch and platformio.ini");
  Serial.println("\nSystem is now running and ready for OTA updates.");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Check if WiFi is still connected
  if (wifiConnected && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost! Attempting to reconnect...");
    wifiConnected = false;
    WiFi.begin(ssid, password);
    connectionStartTime = millis();
  } else if (!wifiConnected && WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("WiFi reconnected!");
    Serial.print("New IP Address: ");
    Serial.println(WiFi.localIP());
  }
  
  // You can add your main program logic here
  
  // Periodically print the IP address (every 30 seconds)
  static unsigned long ipPrintTimer = 0;
  if (Wait(30000, &ipPrintTimer)) {
    Serial.print("Current IP Address: ");
    Serial.println(WiFi.localIP());
  }
} 