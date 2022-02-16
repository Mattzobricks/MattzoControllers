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

// Built-in EEPROM library
// Install via the built-in Library Manager of the Arduino IDE
#include <EEPROM.h>

// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

// Arduino OTA library
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V1.0.5
#include <ArduinoOTA.h>  // Over the air update library

// TinyXML2
// Download from https://github.com/leethomason/tinyxml2
// These files need to be placed in the Arduino library folder:
//   "tinyxml2.cpp"
//   "tinyxml2.h".
// Tested with version of date 2020-11-21.
#include <tinyxml2.h>
using namespace tinyxml2;

// Syslog library
// Download from https://github.com/arcao/Syslog
// Tested with version of Dec 15th, 2020
// Unzip the Syslog folder into the Arduino library folder
#include <WiFiUdp.h>  // Library required for syslog
#include <Syslog.h>  // Syslog library

// MattzoController network configuration
#include "../conf/my/network_config.h"

// Header file
#include "MattzoController_Library.h"


// ****************
// EEPROM functions
// ****************

// EEPROM ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
String eepromIDString = "MattzoController";
// mattzoControllerId. Read from EEPROM when booting the controller.
unsigned int mattzoControllerId;

int getMattzoControllerId() {
  int i;
  int controllerNoHiByte;
  int controllerNoLowByte;

  // set-up EEPROM for read/write operations
  EEPROM.begin(512);

  // Check if the first part of the memory is filled with the MattzoController ID string.
  // This is the case if the controller has booted before with a MattzoController firmware.
  bool idStringCheck = true;
  for (i = 0; i < eepromIDString.length(); i++) {
    char charEeprom = EEPROM.read(i);
    char charIDString = eepromIDString.charAt(i);
    if (charEeprom != charIDString) {
      idStringCheck = false;
      break;
    }
  }

  int paramsStartingPosition = eepromIDString.length();
  if (idStringCheck) {
    // load controller number from preferences
    controllerNoHiByte = EEPROM.read(paramsStartingPosition);
    controllerNoLowByte = EEPROM.read(paramsStartingPosition + 1);
    mattzoControllerId = controllerNoHiByte * 256 + controllerNoLowByte;
    Serial.println("Loaded mattzoControllerId from EEPROM: " + String(mattzoControllerId));
  }
  else {
    // this runs only a single time when starting the controller for the first time

    // Wait a bit to give the user some time to open the serial console...
    delay(5000);

    // store EEPROM ID String in EEPROM
    Serial.println("Initializing controller preferences on first start-up...");
    for (i = 0; i < eepromIDString.length(); i++) {
      EEPROM.write(i, eepromIDString.charAt(i));
    }

    // assign random controller number between MIN_CONTROLLER_ID and MAX_CONTROLLER_ID and store in EEPROM
    mattzoControllerId = random(MIN_CONTROLLER_ID, MAX_CONTROLLER_ID);
    controllerNoHiByte = mattzoControllerId / 256;
    controllerNoLowByte = mattzoControllerId % 256;
    EEPROM.write(paramsStartingPosition, controllerNoHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerNoLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

    Serial.println("Assigned random controller no " + String(mattzoControllerId) + " and stored to EEPROM");
  }

  return mattzoControllerId;
}

// ********************
// Status LED functions
// ********************

enum struct MCConnectionStatus {
  CONNECTING_WIFI = 0x0,
  CONNECTING_MQTT = 0x1,
  CONNECTED = 0x2
};

MCConnectionStatus getConnectionStatus();

// Status LED state.
// If untouched: false (LED is off in normal operation).
// Controllers may use the LED state to indicate specific operation stati
// Example: SensorController indicates sensor contact with the status LED
bool statusLEDState = false;

void setStatusLED(bool ledState) {
  if (STATUS_LED_PIN_INSTALLED) {
    digitalWrite(STATUS_LED_PIN, (ledState ^ STATUS_LED_REVERSE) ? HIGH : LOW);
  }
}

void updateStatusLED() {
  unsigned long t = millis();

  switch (getConnectionStatus()) {
  case MCConnectionStatus::CONNECTING_WIFI:
    // flash
    setStatusLED(((t % 1000) < 100) ^ statusLEDState);
    break;
  case MCConnectionStatus::CONNECTING_MQTT:
    // blink
    setStatusLED(((t % 1000) < 500) ^ statusLEDState);
    break;
  case MCConnectionStatus::CONNECTED:
    // off
    setStatusLED(statusLEDState);
    break;
  }
}


// **************
// Wifi functions
// **************

WiFiClient wifiClient;
char hostname_char[50];

// Status of the Wifi connection (true == "connected")
bool lastKnownWifiConnectedStatus = false;

// Setup wifi parameters and initiate connection process
void setupWifi(bool addMattzoControllerIdToHostname) {
  String hostname_String;

  delay(10);
  if (addMattzoControllerIdToHostname) {
    hostname_String = String(MC_HOSTNAME) + "-" + String(mattzoControllerId);
  } else {
    hostname_String = String(MC_HOSTNAME);
  }
  hostname_String.toCharArray(hostname_char, hostname_String.length() + 1);

  Serial.println("Connecting as " + hostname_String + " to Wifi " + String(WIFI_SSID));
#if defined(ESP8266)
  WiFi.hostname(hostname_char);
#elif defined(ESP32)
  // The following code SHOULD work without the disconnect and config lines, 
  // but it doesn't do its job for some ESP32s.
  // see https://github.com/espressif/arduino-esp32/issues/2537
  WiFi.disconnect(true);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname_char);
#else
#error "Error: this sketch is designed for ESP8266 or ESP32 only."
#endif

  reconnectWiFi();
}

unsigned long lastWiFiBegin_ms = 0;
const unsigned long WAIT_BETWEEN_WIFI_CONNECTS_MS = 500;

// (re)connect to WiFi. Called by checkWifi().
void reconnectWiFi() {
  if ((millis() > lastWiFiBegin_ms + WAIT_BETWEEN_WIFI_CONNECTS_MS) || !lastWiFiBegin_ms) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println(".");
    lastWiFiBegin_ms = millis();
  }
}

// Check and monitor wifi connection status changes
// If not connected, (re)connect!
void checkWifi() {
  if (WiFi.status() != WL_CONNECTED && lastKnownWifiConnectedStatus) {
    lastKnownWifiConnectedStatus = false;
    Serial.println("Connection to Wifi " + String(WIFI_SSID) + " lost.");
  }
  else if (WiFi.status() == WL_CONNECTED && !lastKnownWifiConnectedStatus) {
    lastKnownWifiConnectedStatus = true;
    mcLog("Wifi connected. My IP address is " + WiFi.localIP().toString() + ".");

    // Start OTA listener
    ArduinoOTA.setHostname(MC_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
  }
}


// **************
// MQTT functions
// **************

PubSubClient mqttClient(wifiClient);
unsigned long lastMQTTConnectionAttempt = 0;
#define MQTT_CONNECTION_INTERVAL 3000  // retry to connect 3 seconds after last attempt

// Controller must implement this function
void mqttCallback(char* topic, byte* payload, unsigned int length);

// Setup mqtt parameters
void setupMQTT() {
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);
  mqttClient.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);
}

// Check mqtt connection and initiate reconnection if required
void reconnectMQTT() {
  if (!mqttClient.connected() && (millis() - lastMQTTConnectionAttempt >= MQTT_CONNECTION_INTERVAL)) {
    mcLog("(Re)connecting to MQTT " + String(MQTT_BROKER_IP) + "...");

    String lastWillMessage;
    if (TRIGGER_EBREAK_UPON_DISCONNECT) {
      lastWillMessage = "<sys cmd=\"ebreak\" source=\"lastwill\" mc=\"" + String(MC_HOSTNAME) + "\"/>";
    } else {
      lastWillMessage = "<info msg=\"mc_disconnected\" source=\"lastwill\" mc=\"" + String(MC_HOSTNAME) + "\"/>";
    }
    char lastWillMessage_char[lastWillMessage.length() + 1];
    lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

    setStatusLED(true);
    if (mqttClient.connect(hostname_char, "rocrail/service/command", 0, false, lastWillMessage_char)) {
      setStatusLED(false);
      mqttClient.subscribe("rocrail/service/command");
      mcLog("MQTT connected, listening on topic [rocrail/service/command].");
    } else {
      mcLog("Failed to connect to mqtt, state=" + String(mqttClient.state()));
    }
    lastMQTTConnectionAttempt = millis();
  }
}


// ******************************
// General connectivity functions
// ******************************

MCConnectionStatus getConnectionStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    return MCConnectionStatus::CONNECTING_WIFI;
  }
  if (!mqttClient.connected()) {
    return MCConnectionStatus::CONNECTING_MQTT;
  }
  return MCConnectionStatus::CONNECTED;
}


// ****************
// Syslog functions
// ****************

// A UDP instance for sending and receiving packets over UDP
WiFiUDP udpClient;
// Create a new empty syslog instance
Syslog syslog(udpClient, SYSLOG_PROTO_IETF);

// Setup syslog
void setupSysLog() {
  if (SYSLOG_ENABLED) {
    syslog.server(SYSLOG_SERVER, SYSLOG_PORT);
    syslog.deviceHostname(MC_HOSTNAME);
    syslog.appName(SYSLOG_APP_NAME);
    syslog.defaultPriority(LOG_KERN);
    syslog.logMask(LOG_UPTO(LOG_DEBUG));
    syslog.log(LOG_INFO, "Syslog setup done.");
  }
}

// log a message with a specific severity
void mcLog2(String msg, int severity) {
  if (severity <= LOGLEVEL_SERIAL) {
#if defined(ESP32)
    Serial.print("[");
    Serial.print(xPortGetCoreID());
    Serial.print("] ");
#endif
    Serial.println(msg);
  }
  if (SYSLOG_ENABLED && severity <= LOGLEVEL_SYSLOG) {
    syslog.log(severity, msg);
    delay(1);
    // delay a microsecond as udp packets get dropped on an esp8266 if they happen too close together
  }
}

// log a message with default severity
void mcLog(String msg) {
  mcLog2(msg, LOG_INFO);
}


// ***********************
// setup and loop function
// ***********************

void setupMattzoController(bool addMattzoControllerIdToHostname) {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("MattzoController booting...");

  if (STATUS_LED_PIN_INSTALLED) {
    pinMode(STATUS_LED_PIN, OUTPUT);
  }
  randomSeed(ESP.getCycleCount());
  mattzoControllerId = getMattzoControllerId();
  setupWifi(addMattzoControllerIdToHostname);
  setupSysLog();
  setupMQTT();

  mcLog("MattzoController setup completed.");
}

void loopMattzoController() {
  checkWifi();
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    if (mqttClient.connected()) {
      mqttClient.loop();
    }
    ArduinoOTA.handle();
  }
  updateStatusLED();
}
