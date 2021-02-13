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



// ***********
// MattzoLocos
// ***********

// Number of locos (aka. MattzoLocos) controlled by this controller
const int NUM_LOCOS = 4;

// List of MattzoLocos
// The parameters have the following meaning:
// - locoName: name of the loco as setup in Rocrail
// - locoAddress: address of the loco as setup in Rocrail
// - accelerationInterval: time interval for acceleration / braking (default: 100 ms)
// - accelerateStep: power increment for each acceleration step
// - brakeStep: : power decrement for each braking step
MattzoLocoConfiguration* getMattzoLocoConfiguration() {
  static MattzoLocoConfiguration locoConf[NUM_LOCOS];

  locoConf[0] = (MattzoLocoConfiguration){
  .locoName = "BROCO",
  .locoAddress = 10277,
  .accelerationInterval = 100,
  .accelerateStep = 2,
  .brakeStep = 2
  };

  locoConf[1] = (MattzoLocoConfiguration) {
    .locoName = "ICE",
    .locoAddress = 6051,
    .accelerationInterval = 100,
    .accelerateStep = 2,
    .brakeStep = 3
  };

  locoConf[2] = (MattzoLocoConfiguration) {
  .locoName = "EST",
  .locoAddress = 6197,
  .accelerationInterval = 100,
  .accelerateStep = 2,
  .brakeStep = 3
  };

  locoConf[3] = (MattzoLocoConfiguration) {
  .locoName = "BUS",
  .locoAddress = 98,
  .accelerationInterval = 100,
  .accelerateStep = 2,
  .brakeStep = 2
  };

  return locoConf;
}


// ***************
// Powered Up hubs
// ***************

// Number of Powered Up hubs that shall connect to this controller
const int NUM_HUBS = 6;

// List of Powered Up hubs that shall connect to this controller
MattzoPUHubConfiguration* getMattzoPUHubConfiguration() {
  static MattzoPUHubConfiguration hubConf[NUM_HUBS];

  hubConf[0] = (MattzoPUHubConfiguration){
    .hubName = "BROCO",
    .macAddress = "90:84:2b:0f:ac:c7",
    .devicePortA = MattzoPUDevice::NONE,
    .configMotorA = 0,
    .devicePortB = MattzoPUDevice::PU_MOTOR,
    .configMotorB = 1,
    .locoAddress = 10277
  };

  hubConf[1] = (MattzoPUHubConfiguration){
      .hubName = "ICE1", 
      .macAddress = "90:84:2b:16:15:f8",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::PU_LIGHT,
      .configMotorB = 0,
      .locoAddress = 6051
  };

  hubConf[2] = (MattzoPUHubConfiguration){
      .hubName = "ICE2",
      .macAddress = "90:84:2b:17:e9:4c",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0,
      .locoAddress = 6051
  };

  hubConf[3] = (MattzoPUHubConfiguration){
      .hubName = "EST1",
      .macAddress = "90:84:2b:18:f2:52",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0,
      .locoAddress = 6197
  };

  hubConf[4] = (MattzoPUHubConfiguration){
      .hubName = "EST2",
      .macAddress = "90:84:2b:18:f7:75",
      .devicePortA = MattzoPUDevice::PU_MOTOR,
      .configMotorA = 1,
      .devicePortB = MattzoPUDevice::NONE,
      .configMotorB = 0,
      .locoAddress = 6197
    };

  hubConf[5] = (MattzoPUHubConfiguration){
    .hubName = "BUS",
    .macAddress = "90:84:2b:21:71:46",
    .devicePortA = MattzoPUDevice::PU_MOTOR,
    .configMotorA = 1,
    .devicePortB = MattzoPUDevice::NONE,
    .configMotorB = 0,
    .locoAddress = 98
  };

  return hubConf;
}


// ***************************
// Controller wiring specifics
// ***************************

// NUM_FUNCTIONS represents the number of Rocrail functions that are defined for this controller
// If changed, the number of array values for FUNCTION_PIN below must be changed as well.
// You should also check void lightEvent() in MTC4PU.ino, which is responsible for switching headlights from white to red etc.
const int NUM_FUNCTIONS = 4;

// Digital pins for function output
// For powered up lights, use virtual function pin PU_LIGHT
uint8_t FUNCTION_PIN[NUM_FUNCTIONS] = { PU_LIGHT, 22, 4, 2 };

// The loco address for which the function pin will be triggered.
// You may fill that array up with zeros (0). Meaning: "all trains". Makes only sense if this controller is handling a single train only.
int FUNCTION_PIN_LOCO_ADDRESS[NUM_FUNCTIONS] = { 6051, 6051, 6051, 6051 };

// Automatic lights. If set to true, the lights are switched on when loco is moving forward, and switched off if the train stops or goes backwards.
// To set-up more advanced behaviour, find the lightEvent() function in the MTC4PU code and change it as desired.
const bool AUTO_LIGHTS = false;

// Digital output PIN to monitor controller operation (typically a LED)
bool STATUS_LED_PIN_INSTALLED = true;  // set to true if LED is installed (if not: false)
uint8_t STATUS_LED_PIN = 0;


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
