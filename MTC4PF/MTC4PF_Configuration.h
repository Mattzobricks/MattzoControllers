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



// *************
// Loco specifics
// *************

// Rocrail address of the train
const int LOCO_ADDRESS = 8984;


// ***************************
// Controller wiring specifics
// ***************************

// Type of motor shield installed.
const MotorShieldType MOTORSHIELD_TYPE = MotorShieldType::L9110;

// Constants for motor shield type L298N
#define enA D1  // PWM signal for motor A. Relevant for L298N only.
#define enB D5  // PWM signal for motor B. Relevant for L298N only.

// Constants for motor shield type L298N and L9110
#define in1 D2  // motor A direction control (forward).
#define in2 D3  // motor A direction control (reverse).
#define in3 D6  // motor B direction control (forward).
#define in4 D7  // motor B direction control (reverse).
#define CONFIG_MOTOR_A 1   // configuration for Motor A. 1 = forward, 0 = not installed / not used, -1 = reverse.
#define CONFIG_MOTOR_B -1  // configuration for Motor B. 1 = forward, 0 = not installed / not used, -1 = reverse.

// Constants for motorshield type Lego IR Receiver 8884
#define IR_LED_PIN D1			// pin on which the IR LED is installed.
#define IR_CHANNEL 0			// channel number selected on the Lego IR Receiver 8884. May be 0, 1, 2 or 3.
#define IR_PORT_RED 1     // Usage of red  port on Lego IR Receiver 8884: 1 = motor, default rotation; 0 = no motor connected; -1 = motor, reversed rotation
#define IR_PORT_BLUE 0    // Usage of blue port on Lego IR Receiver 8884: 1 = motor, default rotation; 0 = no motor connected; -1 = motor, reversed rotation

// NUM_FUNCTIONS represents the number of Rocrail functions that are defined for this controller
// If increased, the fn1, fn2... defintions must be enhanced as well.
// Also check for usage of those parameters and extend code accordingly! You should also check void lightEvent(), which is responsible for switching headlights from white to red etc.
const int NUM_FUNCTIONS = 2;

// Digital pins for function output
uint8_t FUNCTION_PIN[NUM_FUNCTIONS] = {D0, D1};




// ***************
// Syslog settings
// ***************

// Syslog application name
const char* SYSLOG_APP_NAME = "MTC4PF";
