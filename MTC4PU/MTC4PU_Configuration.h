// MattzoController Network Configuration
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// This file needs to stay in the folder of the firmware for the specific MattzoController!

// Handling of different configurations:
//		It's a good idea to create copies for all the controllers of this type that you own.
//		Before compiling and uploading the firmware, just include the correct configuration into the firmware code.
//		This allows to manage specific configurations for the different controllers easily.

// Best practice:
// 1. Create a copy of this file if required (see above).
// 2. Go through the settings below and update the settings as required.



// **************
// Loco specifics
// **************

// Rocrail address of the train
const int LOCO_ADDRESS = 8984;


// ***************
// Powered Up hubs
// ***************

// Number of Powered Up hubs that shall connect to this controller
const int NUM_HUBS = 8;

// List of Powered Up hubs that shall connect to this controller
MattzoPUHubConfiguration* getMattzoPUHubConfiguration() {
  static MattzoPUHubConfiguration hubConf[NUM_HUBS];

  hubConf[0] = (MattzoPUHubConfiguration){
      .hubName = "GRECO",
      .macAddress = "90:84:2b:21:71:46",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = -1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0
    };

  hubConf[1] = (MattzoPUHubConfiguration){
      .hubName = "BANAAN1",
      .macAddress = "90:84:2b:01:20:f8",
      .devicePortA = MattzoPUDevice::NONE,
      .configMotorA = 0,
      .devicePortB = MattzoPUDevice::PU_MOTOR,
      .configMotorB = 1
    };

  hubConf[2] = (MattzoPUHubConfiguration){
      .hubName = "ICE1",
      .macAddress = "90:84:2b:16:15:f8",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::PU_LIGHT,
      .configMotorB = 0
    };

  hubConf[3] = (MattzoPUHubConfiguration){
      .hubName = "ICE2",
      .macAddress = "90:84:2b:17:e9:4c",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0
    };

  hubConf[4] = (MattzoPUHubConfiguration){
      .hubName = "BROCO",
      .macAddress = "90:84:2b:0f:ac:c7",
      .devicePortA = MattzoPUDevice::NONE,
      .configMotorA = 0,
      .devicePortB = MattzoPUDevice::PU_MOTOR,
      .configMotorB = 1
    };

  hubConf[5] = (MattzoPUHubConfiguration){
      .hubName = "EST1",
      .macAddress = "90:84:2b:18:f2:52",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0
    };

  hubConf[6] = (MattzoPUHubConfiguration){
      .hubName = "EST2",
      .macAddress = "90:84:2b:18:f7:75",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0
    };

  hubConf[7] = (MattzoPUHubConfiguration){
      .hubName = "BANAAN2",
      .macAddress = "90:84:2b:00:5d:bb",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0
    };

  return hubConf;
}


// ***************************
// Controller wiring specifics
// ***************************

// Digital output PIN to monitor controller operation (typically a LED)
bool STATUS_LED_PIN_INSTALLED = true;  // set to false if no LED is installed
uint8_t STATUS_LED_PIN = 4;


// ****************
// Network settings
// ****************

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT false


// ***************
// Syslog settings
// ***************
// Syslog application name
const char* SYSLOG_APP_NAME = "MTC4PU";
