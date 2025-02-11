#include "Syslog.h"

// MattzoController Network Configuration
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// This file contains the network configuration for the firmware
// You need to adapt it to your specific network environment


// *************
// WiFi settings
// *************

// The SSID of your WiFi network
const char* WIFI_SSID = "railnet";

// The passphrase of your WiFi network
const char* WIFI_PASSWORD = "born2rail";


// *************
// MQTT settings
// *************

// The IP address or hostname of the host on which your MQTT broker (e.g. mosquitto) is running.
const char* MQTT_BROKER_IP = "192.168.178.23";
const int MQTT_BROKER_PORT = 1883;

// MQTT keep alive interval (in seconds)
const int MQTT_KEEP_ALIVE_INTERVAL = 10;


// ************
// Log settings
// ************
// The log levels are defined in Syslog.h of the Syslog library:
// LOG_EMERG 0 /* system is unusable */
// LOG_ALERT 1 /* action must be taken immediately */
// LOG_CRIT  2 /* critical conditions */
// LOG_ERR   3 /* error conditions */
// LOG_WARNING 4 /* warning conditions */
// LOG_NOTICE  5 /* normal but significant condition */
// LOG_INFO  6 /* informational */
// LOG_DEBUG 7 /* debug-level messages */
// Log level for serial output. Only message with a priority of the log level or greater will be logged.

const int LOGLEVEL_SERIAL = LOG_INFO;
// Log level for syslog output. Only message with a priority of the log level or greater will be logged.
const int LOGLEVEL_SYSLOG = LOG_INFO;
// Set to true if syslog shall be enabled
const bool SYSLOG_ENABLED = false;
// IP address or hostname of the syslog server
const char* SYSLOG_SERVER = MQTT_BROKER_IP;
// Port of the syslog server
const int SYSLOG_PORT = 514;


// ************
// OTA settings
// ************

// Password for over-the-air firmware updates
// PLEASE CHANGE THE PASSWORD AND KEEP IT SECRET FROM EVIL PLAYMOBIL FANS!
const char* OTA_PASSWORD = "mattzobricks";
