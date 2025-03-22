#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "Everwood"
#define WIFI_PASSWORD "Everwood-Staff"
#define OTA_PASSWORD "esp32s3admin"
#define HOSTNAME "ESP32Stage1"

// Motor Configuration
#define CUT_MOTOR_SPEED 800
#define CUT_MOTOR_ACCEL 1000
#define POSITION_MOTOR_SPEED 800
#define POSITION_MOTOR_ACCEL 1000
#define HOMING_SPEED 500
#define CUT_MOTOR_MAX_STEPS 9000
#define POSITION_MOTOR_MAX_STEPS 5000

// Timing Parameters
#define DEBOUNCE_DELAY 50
#define SWITCH_READING_INTERVAL 20
#define LED_BLINK_INTERVAL 500
#define HOMING_TIMEOUT 10000
#define CUTTING_OPERATION_TIMEOUT 15000

// Peripheral Control
#define SIGNAL_DURATION 1000
#define SUCTION_CHECK_DELAY 2000

// State Machine
#define MAX_HOMING_ATTEMPTS 3

#endif // CONFIG_H 