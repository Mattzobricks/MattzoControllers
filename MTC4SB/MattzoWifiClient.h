// MattzoController Network Configuration
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// This file needs to be copied into the Arduino library folder
// This location of this folder depends on your system. Usually it is located in the Documents folder of the current user
// Example: C:\Users\matth\Documents\Arduino\libraries\MattzoBricks

// Best practice:
// 1. Navigate to Documents
// 2. Find the folder Arduino/libraries
// 3. Create a subfolder called "MattzoBricks"
// 4. Copy this file into the MattzoBricks folder that you just created.


// ******************
// Included libraries
// ******************

// Arduino OTA library
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V1.0.5
#include <ArduinoOTA.h>  // Over the air update library

// WiFi library for ESP-32
#include <WiFi.h>

/// <summary>
/// Class used to connect to WiFi.
/// </summary>
class MattzoWifiClient {

public:
  // Public static members

  /// <summary>
  /// Setup the WiFi client.
  /// </summary>
  //static void Setup(const char* hostName) {
  static void Setup() {
    if (setupCompleted) {
      Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Setup already completed!");
      return;
    }

    if (setupInitiated) {
      Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Setup already initiated!");
      return;
    }
    else {
      setupInitiated = true;
    }

#if defined(ESP8266)
    WiFi.hostname(mattzoControllerName_char);
#elif defined(ESP32)
    // The following code SHOULD work without the disconnect and config lines, 
    // but it doesn't do its job for some ESP32s.
    // see https://github.com/espressif/arduino-esp32/issues/2537
    WiFi.disconnect(true);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(mattzoControllerName_char);
#else
#error "Error: this sketch is designed for ESP8266 or ESP32 only."
#endif

    Serial.print("[" + String(xPortGetCoreID()) + "] Wifi: Connecting (SSID: " + String(WIFI_SSID) + ").");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Loop until we actually connect.
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println();

    // Start OTA listener.
    ArduinoOTA.setHostname(mattzoControllerName_char);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();

    // Setup completed.
    setupCompleted = true;
  }

  static void Assert() {
    if (!setupCompleted) {
      Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Setup not completed. Execute .Setup() first.");
      return;
    }

    if (WiFi.status() != WL_CONNECTED && wasConnected) {
      wasConnected = false;
      Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Connection " + String(WIFI_SSID) + " lost. Reconnecting...");
    }

    // Loop until we reconnect.
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }

    if (!wasConnected) {
      wasConnected = true;
      Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Connected (IPv4: " + WiFi.localIP().toString() + ").");
    }

    // Handle any OTA updates.
    ArduinoOTA.handle();
  }

private:

  static bool setupInitiated;
  static bool setupCompleted;
  static bool wasConnected;
};

// Initialize static members.
bool MattzoWifiClient::setupInitiated = false;
bool MattzoWifiClient::setupCompleted = false;
bool MattzoWifiClient::wasConnected = false;
