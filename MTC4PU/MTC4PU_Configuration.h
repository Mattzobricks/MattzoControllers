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

// Number of Powered Up hubs that connect to this controller
const int NUM_HUBS = 4;


void initMattzoPUHubs() {
	myHubs[0].initMattzoPUHub(
		"GRECO",
		"90:84:2b:21:71:46",
		MattzoPUDevice::PU_MOTOR, -1,
		MattzoPUDevice::NONE, 0
	);

	myHubs[1].initMattzoPUHub(
		"BANAAN1",
		"90:84:2b:01:20:f8",
		MattzoPUDevice::NONE, 0,
		MattzoPUDevice::PU_MOTOR, 1
	);

	myHubs[2].initMattzoPUHub(
		"ICE1",
		"90:84:2b:16:15:f8",
		MattzoPUDevice::PU_MOTOR, 1,
		MattzoPUDevice::PU_LIGHT, 0
	);

	myHubs[3].initMattzoPUHub(
		"ICE2",
		"90:84:2b:17:e9:4c",
		MattzoPUDevice::PU_MOTOR, 1,
		MattzoPUDevice::PU_LIGHT, 0
	);

	// Mattzes private notes - disregard...
	// {"ICE1", "90:84:2b:16:15:f8", "false", "A", "B"}
	// {"ICE2", "90:84:2b:17:e9:4c", "false", "A", ""}
	// {"BROCO", "90:84:2b:0f:ac:c7", "false", "B", ""}
	// {"GRECO", "90:84:2b:21:71:46", "false", "A", ""}
	// {"EST1", "90:84:2b:18:f2:52", "false", "A", ""}
	// {"EST2", "90:84:2b:18:f7:75", "false", "A", ""}
	// {"BANAAN1", "90:84:2b:01:20:f8", "false", "A", ""}
	// {"BANAAN2", "90:84:2b:00:5d:bb", "false", "A", ""}
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
