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
// - If a USE_PCA9685 port expander is connected to your ESP8266, set USE_PCA9685 to true.
// Wiring:
// - PCA9685 is usually connected to pins D1 (clock) and D2 (data) of the ESP8266.
// - VCC is sourced from V3V of the ESP8266.
// - V+ is sourced from VIN of the ESP8266
// -- VIN should be between 5 and 6 Volts.
// -- According to the documentation, the PCA9685 will also sustain 12V, but this is not recommended and we have not tested it.
// -- We have tested run the board with standard USV voltage (4,65V) and it worked without problems.
// - Connecting GND is mandatory.
// - OE should also be connected. If pulled high, servo power is switched off. Good to preserve your servos. Cabling is usually easiest if pin D0 is used as OE.
// Chaining:
// - Both PCA9685 and the firmware support chaining.
// - Board 1 has the address 0x40, the second one 0x41 etc.
// Servos:
// - Just connect the servos as designed. Servos connected to PCA9685 must be mapped in SWITCHPORT_PIN_PCA9685.
// Signals:
// - Connecting TrixBrix signals to the PCA9685 is somewhat tricky, because of the "common anode" (common plus terminal) of the signal LEDs.
// - The solution is to connect the middle wire (plus) to V3V of the ESP8266 (NOT the plus pins of the PCA9685 ports), and the outer wires to the PCA9685 pins.
// - It is important to remember the correct way of setting a pin on the PCA9685 to:
// -- fully on: pwm.setPWM(port, 4096, 0);
// -- fully off: pwm.setPWM(port, 0, 4096);
// Additional reference: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all


// PCA9685 WIRING CONFIGURATION

// PCA9685 port expander used?
#define USE_PCA9685 false

// PCA9685 OE pin supported?
bool PCA9685_OE_PIN_INSTALLED = false;  // set to true if OE pin is connected (false if not)
uint8_t PCA9685_OE_PIN = D0;

// Number of chained PCA9685 port extenders
#define NUM_PCA9685s 1


// Infos for I/O port expander MCP23017
// Usage:
// - If a MCP23017 I/O port expander is connected to your ESP8266, set USE_MCP23017 to true.
// Wiring:
// - SCL and SDA are usually connected to pins D1 (clock) and D2 (data) of the ESP8266.
// - VCC is sourced from V3V of the ESP8266.
// - GND is connected to GND of the ESP8266 (mandatory!).
// - RESET is connected with an 10K resistor to VCC
// - Address ports A0, A1 and A2 according to the desired address (0x20, 0x21, ...). All connected to GND means address 0x20.
// Ports:
// - The ports of the are numbered as follows:
// - A0..A7: 0..7
// - B0..B7: 8..15
// Chaining:
// - Both MCP23017 and the firmware support chaining.
// - Board 1 has the address 0x20, the second one 0x21 etc.
// - Up to 8 boards can be connected.
// Sensors:
// - Connecting sensors to the MCP23017 is simple.
// - Just connect one of of the cable pair to GND, the other one to one of the ports of the MCP23017.


// MCP23017 WIRING CONFIGURATION

// MCP23017 port expander used?
#define USE_MCP23017 false

// Number of chained PCA9685 port extenders
#define NUM_MCP23017s 1


// SWITCH WIRING CONFIGURATION

// PHYSICAL SWITCH PORTS
// Number of physical switch ports
const int NUM_SWITCHPORTS = 4;

// Digital output pins for switch servos (pins like D0, D1 etc. for ESP-8266 I/O pins, numbers like 0, 1 etc. for pins of the PCA9685)
uint8_t SWITCHPORT_PIN[NUM_SWITCHPORTS] = { D0, D1, D2, D3 };

// Type of digital output pins for switch servos
// 0   : pins on the ESP-8266
// 0x40: ports on the 1st PCA9685
// 0x41: ports on the 2nd PCA9685
// 0x42: ports on the 3rd PCA9685 etc.
uint8_t SWITCHPORT_PIN_TYPE[NUM_SWITCHPORTS] = { 0, 0, 0, 0 };

// LOGICAL SWITCH PORTS
// Number of logical switch ports
const int NUM_LOGICAL_SWITCHPORTS = 4;

// Logical switch ports
// Purpose: required for TrixBrix double slip switches
// Rules:
// - First logical switch port number is always 1001!
int LOGICAL_SWITCHPORTS[NUM_LOGICAL_SWITCHPORTS] = { 1001, 1002, 1003, 1004 };

// Mappings for logical switch ports to physical switch ports
// 1 logical switch port maps to exactly two physical switch ports.
// The values in the array mean the array index of the physical port in the SWITCHPORT_PIN, NOT THE PIN NUMBER!
// Standard configuration:
// - logical port 1001 -> physical switch ports index 0 and 1 (as defined in SWITCHPORT_PIN array)
// - logical port 1002 -> physical switch ports index 2 and 3
// - logical port 1003 -> physical switch ports index 4 and 5
// - logical port 1004 -> physical switch ports index 6 and 7
uint8_t LOGICAL_SWITCHPORT_MAPPINGS[NUM_LOGICAL_SWITCHPORTS * 2] = { 0, 1, 2, 3, 4, 5, 6, 7 };

// Reverse second physical port?
boolean LOGICAL_SWITCHPORT_REV_2ND_PORT[NUM_LOGICAL_SWITCHPORTS] = { false, false, false, false };


// SIGNAL WIRING CONFIGURATION

// Number of signal ports (the number of signal LEDs, not the number of signals!)
const int NUM_SIGNALPORTS = 4;

// Digital pins for signal LEDs (pins like D0, D1 etc. for ESP-8266 I/O pins, numbers like 0, 1 etc. for pins of the PCA9685)
uint8_t SIGNALPORT_PIN[NUM_SIGNALPORTS] = { D4, D5, D6, D7 };

// Type of digital output pins for signal LEDs
// 0   : pin on the ESP-8266
// 0x40: port on the 1st PCA9685
// 0x41: port on the 2nd PCA9685
// 0x42: port on the 3rd PCA9685 etc.
uint8_t SIGNALPORT_PIN_TYPE[NUM_SIGNALPORTS] = { 0, 0, 0, 0 };


// SENSOR WIRING CONFIGURATION

// Set to true to enable remote sensors.
// Remote sensors are not electrically connected to this controller, they are triggered via Rocrail commands.
// Remote sensors can be used for level crossings in Autonomous Mode
// If you do not control a level crossing in Autonomous Mode with this controller, set to false!
const bool REMOTE_SENSORS_ENABLED = false;

// Number of sensors connected or connectable to the controller
const int NUM_SENSORS = 0;

// Digital input PINs for hall, reed or other digital sensors (pins like D0, D1 etc. for ESP-8266 I/O pins, numbers like 0, 1 etc. for pins of the MCP23017)
// If sensor is a remote sensor, enter the "Address" of the sensor in Rocrail.
// If sensor is a virtual sensor, the value has no meaning (e.g. set to zero).
uint8_t SENSOR_PIN[NUM_SENSORS] = { };

// Type of digital input pins for sensors
// 0   : pin on the ESP-8266
// 0x10: remote sensor, triggered via Rocrail message (REMOTE_SENSOR_PIN_TYPE)
// 0x11: virtual sensor, triggered when bascule bridge is fully open or closed.
// 0x20: local sensor, connected to a port on the 1st MCP23017
// 0x21: local sensor, connected to a port on the 2nd MCP23017
// 0x22: local sensor, connected to a port on the 3rd MCP23017 etc.
const int LOCAL_SENSOR_PIN_TYPE = 0;
const int REMOTE_SENSOR_PIN_TYPE = 0x10;
const int VIRTUAL_SENSOR_PIN_TYPE = 0x11;
const int MCP23017_SENSOR_PIN_TYPE = 0x20;
uint8_t SENSOR_PIN_TYPE[NUM_SENSORS] = { };

// If sensor is a remote sensor, the MattzoControllerId of the MattzoController to which the sensor is connected must be entered into this array.
// If sensor is local or virtual, the value has no meaning (e.g. set to zero)
int SENSOR_REMOTE_MATTZECONTROLLER_ID[NUM_SENSORS] = { };


// STATUS LED WIRING CONFIGURATION

// Digital output pin to monitor controller operation (typically a LED)
bool STATUS_LED_PIN_INSTALLED = true;  // set to false if no LED is installed
uint8_t STATUS_LED_PIN = D8;


// LEVEL CROSSING CONFIGURATION

// General switch for level crossing (false = no level crossing connected; true = level crossing connected)
const bool LEVEL_CROSSING_CONNECTED = true;

// Port configured for the level crossing in Rocrail
// Attention: make sure the port does not conflict with a switch port!
// -1 indicates "no port" = "does not accept rocrail commands"
const int LEVEL_CROSSING_RR_PORT = 1;

// LEVEL CROSSING BOOM BARRIERS

// Number of boom barrier servos configured for the level crossing
const int LC_NUM_BOOM_BARRIERS = 4;

// Servo ports (indices in the SWITCHPORT_PIN array)
// servo 1 and 2 represents primary barriers, servo 3 and subsequent servos represents secondary barriers
uint8_t LC_BOOM_BARRIER_SERVO_PIN[LC_NUM_BOOM_BARRIERS] = { 0, 1, 2, 3 };

// Closing timespan for all boom barriers
const unsigned int LC_BOOM_BARRIER_CLOSING_PERIOD_MS = 2500;
// Delay until primary boom barriers start closing
const unsigned int LC_BOOM_BARRIER1_CLOSING_DELAY_MS = 2000;
// Delay until secondary boom barriers start closing
const unsigned int LC_BOOM_BARRIER2_CLOSING_DELAY_MS = 4000;
// Opening timespan for all boom barriers
const unsigned int LC_BOOM_BARRIER_OPENING_PERIOD_MS = 3000;

// Servo angles for "up" and "down" positions (primary booms)
// Approximate angles for TrixBrix boom barrier servos to start with:
// - If servo is connected directly to the ESP8266:
// -- Right hand traffic: 0, left hand traffic: 180
// - If servo is connected to PCA9685:
// -- Right hand traffic: 30, left hand traffic: 143
const unsigned int LC_BOOM_BARRIER_ANGLE_PRIMARY_UP = 0;
const unsigned int LC_BOOM_BARRIER_ANGLE_PRIMARY_DOWN = 87;

// Servo angles for "up" and "down" positions (secondary booms)
// Approximate angles for TrixBrix boom barrier servos to start with:
// - If servo is connected directly to the ESP8266:
// -- Right hand traffic: 180, left hand traffic: 0
// - If servo is connected to PCA9685:
// -- Right hand traffic: 143, left hand traffic: 30
const unsigned int LC_BOOM_BARRIER_ANGLE_SECONDARY_UP = 180;
const unsigned int LC_BOOM_BARRIER_ANGLE_SECONDARY_DOWN = 90;

// LEVEL CROSSING SIGNALS

// Number of signals configured for the level crossing
const int LC_NUM_SIGNALS = 4;

// Signal ports (indices in the SIGNALPORT_PIN array)
uint8_t LC_SIGNAL_PIN[LC_NUM_SIGNALS] = { 0, 1, 2, 3 };

// Signal flash period in milliseconds (full cycle).
const unsigned int LC_SIGNAL_FLASH_PERIOD_MS = 1500;

// Set to true to enable signal fading (brightens and fades lights gradually for enhanced realism)
const bool LC_SIGNALS_FADING = true;

// LEVEL CROSSING SENSORS

// Virtual sensor for "booms closed" feedback event (index in the SENSOR_PIN array). This virtual sensor is triggered after the boom barriers have closed.
// Must be set to -1 to skip virtual "booms closed" sensor event
const int LEVEL_CROSSING_SENSOR_BOOMS_CLOSED = -1;
// Virtual sensor for "booms opened" feedback event (index in the SENSOR_PIN array). This virtual sensor is triggered after the boom barriers have opened.
// Must be set to -1 to skip virtual "booms opened" sensor event
const int LEVEL_CROSSING_SENSOR_BOOMS_OPENED = -1;

// Autonomous Mode enabled?
const bool LC_AUTONOMOUS_MODE = false;

// Number of tracks leading over the level crossing
const int LC_NUM_TRACKS = 2;

// Number of level crossing sensors
const int LC_NUM_SENSORS = 4;

// Sensor array (indices within the SENSOR_PIN array)
// Sensors may occur twice (if sensor is used both as inbound and outbound sensor)
const int LC_SENSORS_INDEX[LC_NUM_SENSORS] = { 0, 1, 2, 3 };

// Track index of the sensor (0: track 1, 1: track 2 etc.)
const int LC_SENSORS_TRACK[LC_NUM_SENSORS] = { 0, 0, 1, 1 };

// Purpose of the sensor (0: inbound / 1: outbound / 2: both)
const int LC_SENSORS_PURPOSE[LC_NUM_SENSORS] = { 2, 2, 2, 2 };

// Orientation of the sensor (0: plus side / 1: minus side)
const int LC_SENSORS_ORIENTATION[LC_NUM_SENSORS] = { 0, 1, 0, 1 };

// Timeout after which axle counters are reset and the track is released (in milliseconds)
const unsigned int LC_AUTONOMOUS_MODE_TRACK_TIMEOUT_MS = 30000;


// BASCULE BRIDGE CONFIGURATION

// General switch for bascule bridge (false = no bridge connected; true = bridge connected)
bool BASCULE_BRIDGE_CONNECTED = false;

// Port configured for the bascule bridge in Rocrail
const int BASCULE_BRIDGE_RR_PORT = 1;

// Motor shield pins for bridge motor direction control
const int BASCULE_BRIDGE_MS_IN1 = D0;  // forward
const int BASCULE_BRIDGE_MS_IN2 = D1;  // reverse

// Motor power settings for bridge operations
const int BASCULE_BRIDGE_POWER_UP = 1023;  // Motor power for pulling the bridge up (0 .. 1023)
const int BASCULE_BRIDGE_POWER_UP2 = 512;  // Motor power for pulling the bridge up after the "bridge up" sensor has been triggered (0 .. 1023)
const int BASCULE_BRIDGE_POWER_DOWN = 1023;  // Motor power for letting the bridge down (0 .. 1023)
const int BASCULE_BRIDGE_POWER_DOWN2 = 512;  // Motor power for closing the bridge down after the "bridge down" sensor has been triggered (0 .. 1023)

// Signal ports (set to -1 for "not connected")
const int BASCULE_BRIDGE_SIGNAL_RIVER_STOP = 0;  // signal port that is activated when bridge is not in the "up" position (index in the SIGNALPORT_PIN array)
const int BASCULE_BRIDGE_SIGNAL_RIVER_PREP = 1;  // signal port that is activated in addition to the "stop" port when bridge is opening (index in the SIGNALPORT_PIN array)
const int BASCULE_BRIDGE_SIGNAL_RIVER_GO = 2;  // signal port that is activated when bridge has reached the "up" position (index in the SIGNALPORT_PIN array)
const int BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT = 3;  // signal port for a blinking light that indicates opening/closing action (index in the SIGNALPORT_PIN array)

// Sensor ports
// local sensor that indiciates "bridge down" (index in the SENSOR_PIN array)
const int BASCULE_BRIDGE_SENSOR_DOWN = 0;
// local sensor that indicates "bridge up" (index in the SENSOR_PIN array)
const int BASCULE_BRIDGE_SENSOR_UP = 1;
// virtual sensor that indiciates "bridge fully down" (index in the SENSOR_PIN array). This virtual sensor is triggered after the "extra time after closed".
// Must be set to -1 to skip virtual "bridge fully down" sensor events
const int BASCULE_BRIDGE_SENSOR_FULLY_DOWN = 2;
// virtual sensor that indicates "bridge fully up" (index in the SENSOR_PIN array). This virtual sensor is triggered after the "extra time after opened".
// Must be set to -1 to skip virtual "bridge fully up" sensor events
const int BASCULE_BRIDGE_SENSOR_FULLY_UP = 3;

// Timings (in milli seconds)
// Maximum allowed time for opening the bridge until the opening sensor must have been triggered. After this time has passed, the bridge motor is stopped for safety reasons.
const unsigned int BASCULE_BRIDGE_MAX_OPENING_TIME_MS = 45000;
// Maximum allowed time for closing the bridge until the closing sensor must have been triggered. After this time has passed, the bridge motor is stopped for safety reasons.
const unsigned int BASCULE_BRIDGE_MAX_CLOSING_TIME_MS = 45000;
// Extra time after the "bridge up" sensor has been triggered until the bridge motor is stopped.
const unsigned int BASCULE_BRIDGE_EXTRA_TIME_AFTER_OPENED_MS = 2000;
// Extra time after the "bridge down" sensor has been triggered until the bridge motor is stopped.
const unsigned int BASCULE_BRIDGE_EXTRA_TIME_AFTER_CLOSED_MS = 500;


// ****************
// Network settings
// ****************

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT false


// ***************
// Syslog settings
// ***************
// Syslog application name
const char* SYSLOG_APP_NAME = "MLC";
