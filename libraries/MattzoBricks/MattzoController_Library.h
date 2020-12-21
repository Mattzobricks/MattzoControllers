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


// Libraries for syslog
#include <WiFiUdp.h>
#include <Syslog.h>

// MattzoController network configuration
// (this following file be placed in the same folder as this file)
#include <MattzoController_Network_Configuration.h>



// ***************
// Some declarions
// ***************

void mcLog(String msg);


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
  digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
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
  Serial.println("Connecting to Wifi " + String(WIFI_SSID));
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
void reconnectMQTT(char *mqttClientName_char) {
  if (!mqttClient.connected() && (millis() - lastMQTTConnectionAttempt >= MQTT_CONNECTION_INTERVAL)) {
    mcLog("(Re)connecting MQTT...");

    String lastWillMessage = String(mqttClientName_char) + " " + "last will and testament";
    char lastWillMessage_char[lastWillMessage.length() + 1];
    lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

    setStatusLED(true);
    if (mqttClient.connect(mqttClientName_char, "roc2bricks/lastWill", 0, false, lastWillMessage_char)) {
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
void sendMQTTPing(PubSubClient* mqttClient, char* pingMsg_char) {
  if (SEND_PING && (millis() - lastPing >= SEND_PING_INTERVAL)) {
    lastPing = millis();
    Serial.println("sending ping...");
    mqttClient->publish("roc2bricks/ping", pingMsg_char);
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
