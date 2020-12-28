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



// ***************************
// Controller wiring specifics
// ***************************

// Infos for port expander PCA9685
// Usage:
// - If a USE_PCA9685 port expander is connected to your ESP8266, set USE_PCA9685 to true
// Wiring:
// - PCA9685 is usually connected to pins D1 (clock) and D2 (data) of the ESP8266
// - VCC is sourced from V3V of the ESP8266
// - V+ is sourced from VIN of the ESP8266 and should be between 5 and 6 Volts. According to the documentation, it will also sustain 12V, but this is not recommended (UNTESTED!)
// - Connecting GND is mandatory
// - OE should also be connected. If pulled high, servo power is switched off. Good to preserve your servos. Cabling is usually easiest if pin D0 is used as OE.
// Chaining:
// - PCA9685 supports chaining. However, this is not supported in this firmware yet. Only board 0x40 is usable at this time.
// Servos:
// - Just connect the servos as designed. Servos connected to PCA9685 must be mapped in SWITCHPORT_PIN_PCA9685.
// Signals:
// - Connecting TrixBrix signals to the PCA9685 is somewhat tricky, because of the "common anode" of the signal.
// - The solution is to connect the middle wire (plus) to V3V of the ESP8266 (NOT the plus pins of the PCA9685 ports), and the outer wires to the PCA9685 pins.
// - It is important to remember the correct way of setting a pin on the PCA9685 to:
// -- fully on: pwm.setPWM(port, 4096, 0);
// -- fully off: pwm.setPWM(port, 0, 4096);
// Additional reference: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all


// PCA9685 WIRING CONFIGURATION

// PCA9685 port expander used?
DEFINE USE_PCA9685 true

// PCA9685 OE pin supported?
bool PCA9685_OE_PIN_INSTALLED = true;  // set to false if OE pin is not connected
uint8_t PCA9685_OE_PIN = D0;


// SWITCH WIRING CONFIGURATION

// Number of switch ports
const int NUM_SWITCHPORTS = 5;

// Digital output pins for switch servos
uint8_t SWITCHPORT_PIN[NUM_SWITCHPORTS] = { 0, 1, 2, 3, 4 };

// Type of digital output pins for switch servos (0 = pins on the ESP-8266; 0x40 = ports of the PCA9685)
uint8_t SWITCHPORT_PIN_TYPE[NUM_SWITCHPORTS] = { 0x40, 0x40, 0x40, 0x40, 0x40 };


// SIGNAL WIRING CONFIGURATION

// Number of signal ports (the number of signal LEDs, not the number of signals!)
const int NUM_SIGNALPORTS = 8;

// Digital pins for signal LEDs
uint8_t SIGNALPORT_PIN[NUM_SIGNALPORTS] = { 8, 9, 10, 11, 12, 13, 14, 15 };

// Type of digital output pins for signal LEDs (0 = pins on the ESP-8266; 0x40 = ports of the PCA9685)
uint8_t SIGNALPORT_PIN_TYPE[NUM_SIGNALPORTS] = { 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40 };


// SENSOR WIRING CONFIGURATION

// Number of sensors connected or connectable to the controller
const int NUM_SENSORS = 5;

// Digital input PINs for hall, reed or other digital sensors
uint8_t SENSOR_PIN[NUM_SENSORS] = {D3, D4, D5, D6, D7};


// STATUS LED WIRING CONFIGURATION

// Digital output pin to monitor controller operation (typically a LED)
bool STATUS_LED_PIN_INSTALLED = true;  // set to false if no LED is installed
uint8_t STATUS_LED_PIN = D8;


// ****************
// Network settings
// ****************

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT true


// ***************
// Syslog settings
// ***************
// Syslog application name
const char* SYSLOG_APP_NAME = "MSTC";
