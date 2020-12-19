// MattzoController Network Configuration
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// This file needs to be MOVED (NOT COPIED!) into the Arduino library folder
// This location of this folder depends on your system. Usually it is located in the Documents folder of the current user
// Example: C:\Users\matth\Documents\Arduino\libraries\MattzoBricks
// Attention: DELETE ALL OTHER COPIES OF THIS FILE TO MAKE SURE THAT THE ARDUINO IDE IS USING THE CORRECT VERSION OF THE FILE!

// Best practice:
// 1. Navigate to Documents
// 2. Find the folder Arduino/libraries
// 3. Create a subfolder called "MattzoBricks" if it does not exist yet.
// 4. Move this file into the MattzoBricks folder that you just created.
// 5. Go through the settings below and update the settings as required.


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
// The IP address of the host on which your MQTT broker (e.g. mosquitto) is running.
const char* MQTT_BROKER_IP = "192.168.1.15";

// MQTT keep alive interval (in seconds)
const int MQTT_KEEP_ALIVE_INTERVAL = 5;

// *************
// Ping settings
// *************
// Attention: pings were deprecated with issue #9 and replaced by MQTT last will messages
// Set to true if pings should be sent
const boolean SEND_PING = false;

// Interval for sending pings in milliseconds (5000 ^= 5 seconds)
const int SEND_PING_INTERVAL = 5000;

// ***************
// Syslog settings
// ***************
// Set to true if sys log shall be enabled
const bool SYSLOG_ENABLED = false;
// IP or hostname of the syslog server
const char* SYSLOG_SERVER = MQTT_BROKER_IP;
// Port of the syslog server
const int SYSLOG_PORT = 514;
