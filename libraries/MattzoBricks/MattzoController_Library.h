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



// **************
// Ping functions
// Attention: pings were deprecated with issue #9 and replaced by mqtt last will messages
// **************

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
