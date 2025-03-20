# ESP32-S3 IP Address Finder

This project sets up an ESP32-S3 to connect to WiFi and display its IP address over Serial. This enables Over-The-Air (OTA) updates for future programming.

## Hardware Requirements

- Freenove ESP32-S3 Breakout Board
- USB-C cable for initial programming

## Pin Layout Reference (ESP32-S3)

**Left side (top to bottom):**
4, 5, 6, 7, 15, 16, 17, 18, 8, 3, 46, 9, 10, 11, 12, 13, 14

**Right side (top to bottom):**
1, 2, 42, 41, 40, 39, 38, 37, 36, 35, 0, 45, 48, 47, 21, 20, 19

## Setup Instructions

1. Connect the ESP32-S3 to your computer via USB-C
2. Open the project in PlatformIO
3. Build and upload the code using USB (Serial)
4. Open the Serial Monitor (115200 baud)
5. The device will connect to WiFi and display its IP address

## OTA Updates

Once you have the IP address, you can enable OTA updates:

1. In `platformio.ini`, uncomment the OTA upload settings section
2. Replace `IP_ADDRESS_WILL_BE_SHOWN` with your device's actual IP (shown in Serial Monitor)
3. Set the OTA password to match the one in your code (default: `esp32s3admin`)
4. For future uploads, PlatformIO will use OTA instead of USB

## WiFi Configuration

The code is pre-configured with the following WiFi credentials:
- SSID: Everwood
- Password: Everwood-Staff

To change these credentials, modify the corresponding variables in `src/main.cpp`.

## Security Notes

- Change the default OTA password (`esp32s3admin`) to something more secure
- Update both the code and `platformio.ini` with your new password

## Troubleshooting

- If WiFi connection fails, the device will enter an infinite loop. Check your credentials.
- For OTA upload issues, ensure you're on the same network as the ESP32-S3
- Verify the IP address hasn't changed (DHCP may assign a new IP after power cycling) 