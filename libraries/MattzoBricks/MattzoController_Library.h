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

// Built-in libraries of the Arduino IDE
#include <EEPROM.h>        // EEPROM library
#include <ESP8266WiFi.h>  // WiFi library

// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

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
// the following file needs to be moved into the Arduino library folder
#include <MattzoController_Network_Configuration.h>


// ********************
// Forward declarations
// ********************

void mcLog(String msg);


// ****************
// EEPROM functions
// ****************

// EEPROM ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
String eepromIDString = "MattzoSignalController";
// length of the ID String. Needs to be updated if the ID String is changed.
const int eepromIDStringLength = 22;
// mattzoControllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
unsigned int mattzoControllerNo;
#define MAX_CONTROLLER_ID 65000
String mattzoControllerName;
char mattzoControllerName_char[eepromIDStringLength + 5 + 1];  // the name of the mqttClient must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

void loadPreferences() {
  int i;
  int controllerNoHiByte;
  int controllerNoLowByte;

  // set-up EEPROM read/write operations
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
    mattzoControllerNo = controllerNoHiByte * 256 + controllerNoLowByte;
    Serial.println("Loaded mattzoControllerNo from EEPROM: " + String(mattzoControllerNo));
  }
  else {
    // preferences not initialized yet -> initialize controller
    // this runs only a single time when starting the controller for the first time

    // Wait a bit to give the user some time to open the serial console...
    delay(5000);

    Serial.println("Initializing controller preferences on first start-up...");
    for (i = 0; i < eepromIDString.length(); i++) {
      EEPROM.write(i, eepromIDString.charAt(i));
    }

    // assign random controller number between 1 and MAX_CONTROLLER_ID and store in EEPROM
    mattzoControllerNo = random(1, MAX_CONTROLLER_ID);
    controllerNoHiByte = mattzoControllerNo / 256;
    controllerNoLowByte = mattzoControllerNo % 256;
    EEPROM.write(paramsStartingPosition, controllerNoHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerNoLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

    Serial.println("Assigned random controller no " + String(mattzoControllerNo) + " and stored to EEPROM");
  }

  // set mattzoControllerName
  mattzoControllerName = eepromIDString + String(mattzoControllerNo);
  mattzoControllerName.toCharArray(mattzoControllerName_char, mattzoControllerName.length() + 1);
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

void setStatusLED(bool ledState) {
  if (STATUS_LED_PIN_INSTALLED) {
    digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
  }
}

void updateStatusLED() {
  unsigned long t = millis();

  switch (getConnectionStatus()) {
  case MCConnectionStatus::CONNECTING_WIFI:
    setStatusLED((t % 1000) < 100);
    break;
  case MCConnectionStatus::CONNECTING_MQTT:
    setStatusLED((t % 1000) < 500);
    break;
  case MCConnectionStatus::CONNECTED:
    setStatusLED(false);
    break;
  }
}


// **************
// Wifi functions
// **************

WiFiClient wifiClient;

// Status of the Wifi connection (true == "connected")
bool lastKnownWifiConnectedStatus = false;

// Setup wifi parameters and initiate connection process
void setupWifi() {
  delay(10);
  Serial.println("Connecting to Wifi " + String(WIFI_SSID) + "...");
  WiFi.hostname(mattzoControllerName_char);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// Check and monitor wifi connection status changes
void checkWifi() {
  if (WiFi.status() != WL_CONNECTED && lastKnownWifiConnectedStatus) {
    lastKnownWifiConnectedStatus = false;
    Serial.println("Connection to Wifi " + String(WIFI_SSID) + " lost.");
  }
  else if (WiFi.status() == WL_CONNECTED && !lastKnownWifiConnectedStatus) {
    lastKnownWifiConnectedStatus = true;
    mcLog("Wifi connected. My IP address is " + WiFi.localIP().toString() + ".");
  }
}


// **************
// MQTT functions
// **************

PubSubClient mqttClient(wifiClient);
unsigned long lastMQTTConnectionAttempt = 0;
#define MQTT_CONNECTION_INTERVAL 3000  // retry to connect 1 sec after last attempt

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

    String lastWillMessage = String(mattzoControllerName_char) + " " + "last will and testament";
    char lastWillMessage_char[lastWillMessage.length() + 1];
    lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

    setStatusLED(true);
    if (mqttClient.connect(mattzoControllerName_char, "roc2bricks/lastWill", 0, false, lastWillMessage_char)) {
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


// **************
// Ping functions
// **************
// Attention: pings were deprecated with issue #9 and replaced by mqtt last will messages

// Time of the last sent ping
unsigned long lastPing = millis();

// Send ping to MQTT
void sendMQTTPing() {
  if (SEND_PING && (millis() - lastPing >= SEND_PING_INTERVAL)) {
    lastPing = millis();
    Serial.println("sending ping...");
    mqttClient.publish("roc2bricks/ping", mattzoControllerName_char);
  }
}


// ****************
// Syslog functions
// ****************

// A UDP instance for sending and receiving packets over UDP
WiFiUDP udpClient;
// Create a new empty syslog instance
Syslog syslog(udpClient, SYSLOG_PROTO_IETF);

// Setup syslog
void setupSysLog(char *deviceHostname) {
  if (SYSLOG_ENABLED) {
    syslog.server(SYSLOG_SERVER, SYSLOG_PORT);
    syslog.deviceHostname(deviceHostname);
    syslog.appName(SYSLOG_APP_NAME);
    syslog.defaultPriority(LOG_KERN);
    syslog.logMask(LOG_UPTO(LOG_INFO));
    syslog.log(LOG_INFO, "Syslog setup done.");
  }
}

// log a message with a specific severity
void mcLog2(String msg, int severity) {
  Serial.println(msg);
  if (SYSLOG_ENABLED) {
    syslog.log(severity, msg);
    delay(1);
    // delay a microsecond as udp packets get dropped on an esp8266 if they happen too close together
  }
}

// log a message with default severity
void mcLog(String msg) {
  mcLog2(msg, LOG_INFO);
}


// ************************
// setup and loop functions
// ************************

void setupMattzoController() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("MattzoController booting...");

  if (STATUS_LED_PIN_INSTALLED) {
    pinMode(STATUS_LED_PIN, OUTPUT);
  }
  randomSeed(ESP.getCycleCount());
  loadPreferences();
  setupWifi();
  setupSysLog(mattzoControllerName_char);
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
      sendMQTTPing();
    }
  }
  updateStatusLED();
}
