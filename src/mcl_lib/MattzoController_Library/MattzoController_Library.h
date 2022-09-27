// MattzoController Network Configuration
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef MattzoController_Library_h
#define MattzoController_Library_h

// Must include here to make platformio happy in finding the includes
#include <ArduinoOTA.h>  // Over the air update library
#include <WiFiUdp.h>  // Library required for syslog
#include <Syslog.h>  // Syslog library
#include <PubSubClient.h>
#include <EEPROM.h>
#include <tinyxml2.h>
using namespace tinyxml2;
// ********************
// Forward declarations
// ********************

void mcLog(String msg);
void mcLog2(String msg, int severity);
void reconnectWiFi();
void checkWifi();

void setupMattzoController(bool addMattzoControllerIdToHostname);
void loopMattzoController();

extern WiFiClient wifiClient;
extern PubSubClient mqttClient;
// *********
// Constants
// *********

// If the mattzoControllerId can not be retrieved from EEPROM, a new mattzoControllerId between MIN_CONTROLLER_ID and MAX_CONTROLLER_ID is generated and stored in EEPROM.
#define MIN_CONTROLLER_ID 10000
#define MAX_CONTROLLER_ID 65000

#endif
