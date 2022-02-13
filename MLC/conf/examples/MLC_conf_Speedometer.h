// MattzoController Configuration
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

// Number of chained MCP23017 port extenders
#define NUM_MCP23017s 1


// U8g2 Display used?
#define USE_U8G2 true


// SERVO WIRING CONFIGURATION

// Number of servos
const int NUM_SERVOS = 0;

struct ServoConfiguration {
  // Digital output pins for switch servos (pins like D0, D1 etc. for ESP-8266 I/O pins, numbers like 0, 1 etc. for pins of the PCA9685)
  uint8_t pin;

  // Type of digital output pins for switch servos
  // 0   : pin on the ESP-8266
  // 0x40: port on the 1st PCA9685
  // 0x41: port on the 2nd PCA9685
  // 0x42: port on the 3rd PCA9685 etc.
  uint8_t pinType;

  // set to true if servo shall be detached from PWM signal a couple of seconds after usage
  // this feature is helpful to prevent blocking servo from burning down, it saves power and reduced servo flattering
  // for bascule bridges, the feature must be switched off!
  bool detachAfterUsage;
} servoConfiguration[NUM_SERVOS] = {};


// LED WIRING CONFIGURATION

// Number of LEDs
// LEDs are used in signals, level crossing lights or bascule bridge lights
// As an example, 2 LEDs are required for a light signal with 2 aspects
const int NUM_LEDS = 0;

struct LEDConfiguration {
  // Digital output pin for signal LED (pins like D0, D1 etc. for ESP-8266 I/O pins, numbers like 0, 1 etc. for pins of the PCA9685)
  uint8_t pin;

  // Type of digital output pins for led
  // 0   : LED output pin on the ESP-8266
  // 0x20: LED port on the 1st MCP23017
  // 0x21: LED port on the 2nd MCP23017
  // 0x22: LED port on the 3rd MCP23017 etc.
  // 0x40: LED port on the 1st PCA9685
  // 0x41: LED port on the 2nd PCA9685
  // 0x42: LED port on the 3rd PCA9685 etc.
  uint8_t pinType;
} ledConfiguration[NUM_LEDS] = {};


// SENSOR WIRING CONFIGURATION

// Set to true to enable remote sensors.
// Remote sensors are not electrically connected to this controller, they are triggered via Rocrail commands.
// Remote sensors can be used for level crossings in Autonomous Mode
// If you do not control a level crossing in Autonomous Mode with this controller, set to false!
#define REMOTE_SENSORS_ENABLED false

// Number of sensors connected or connectable to the controller
#define NUM_SENSORS 2

// Constants for type of digital input pins for sensors
// 0   : local sensor on the ESP-8266 (D0 .. D8)
// 0x10: remote sensor, triggered via Rocrail message (REMOTE_SENSOR_PIN_TYPE)
// 0x11: virtual sensor, triggered when bascule bridge is fully open or closed.
// 0x20: local sensor, connected to a port on the 1st MCP23017
// 0x21: local sensor, connected to a port on the 2nd MCP23017
// 0x22: local sensor, connected to a port on the 3rd MCP23017 etc.
#define LOCAL_SENSOR_PIN_TYPE 0
#define REMOTE_SENSOR_PIN_TYPE 0x10
#define VIRTUAL_SENSOR_PIN_TYPE 0x11
#define MCP23017_SENSOR_PIN_TYPE 0x20

struct SensorConfiguration {
  // Digital input PINs for hall, reed or other digital sensors (pins like D0, D1 etc. for ESP-8266 I/O pins, numbers like 0, 1 etc. for pins of the MCP23017)
  // If sensor is a remote sensor, enter the "Address" of the sensor in Rocrail.
  // If sensor is a virtual sensor, the value has no meaning (set to -1 by convention).
  uint8_t pin;

  // Type of digital input pins for sensors
  uint8_t pinType;

  // If sensor is a remote sensor, the MattzoControllerId of the MattzoController to which the sensor is connected must be entered into this array.
  // If sensor is local or virtual, the value has no meaning (set to -1 by convention)
  int remoteMattzoControllerId;
} sensorConfiguration[NUM_SENSORS] =
{
  {
    .pin = D6,
    .pinType = LOCAL_SENSOR_PIN_TYPE,
    .remoteMattzoControllerId = -1
  },
  {
    .pin = D7,
    .pinType = LOCAL_SENSOR_PIN_TYPE,
    .remoteMattzoControllerId = -1
  },
};


// STATUS LED WIRING CONFIGURATION

// Digital output pin to monitor controller operation (typically a LED)
bool STATUS_LED_PIN_INSTALLED = true;  // set to false if no LED is installed
uint8_t STATUS_LED_PIN = D4;
bool STATUS_LED_REVERSE = true;



// SWITCH CONFIGURATION

// Number of switches
const int NUM_SWITCHES = 0;

struct SwitchConfiguration {
  int rocRailPort;
  int servoIndex;

  // servo2Index: servo index for second servo (required for TrixBrix double slip switches). If unused, set to -1.
  // servo2Reverse: set to true if second servo shall be reversed
  int servo2Index;
  bool servo2Reverse;

  // feedback sensors
  // set triggerSensors to true if used
  // The first value in the sensorIndex is the index of the virtual sensor in the sensor array for the "straight" sensor, the second is for "turnout".
  // Both referenced sensors must be virtual sensors.
  bool triggerSensors;
  int sensorIndex[2];
} switchConfiguration[NUM_SWITCHES] = {};


// SIGNAL CONFIGURATION

// Number of signals
const int NUM_SIGNALS = 0;
// Maximum number of signal aspects (e.g. red, green, yellow)
const int NUM_SIGNAL_ASPECTS = 2;
// Number of signal LEDs (usually equal to NUM_SIGNAL_ASPECTS)
const int NUM_SIGNAL_LEDS = 2;
// Maximum number of servos for form signals (e.g. one for the primary and another one for the secondary semaphore)
// If no form signals are used, just set to 0
const int NUM_SIGNAL_SERVOS = 0;

struct SignalConfiguration {
  // the port configured in Rocrail for an aspect
  // 0: aspect not supported by this signal
  int aspectRocrailPort[NUM_SIGNAL_ASPECTS];
  // if a LED is configured for this aspect (this is the usual case for light signals), this value represents the index of the LED in the SIGNALPORT_PIN array.
  // -1: no LED configured for this aspect
  int aspectLEDPort[NUM_SIGNAL_LEDS];
  // mappings between aspects and LEDs (often a diagonal matrix)
  // true: LED is mapped for this aspect
  bool aspectLEDMapping[NUM_SIGNAL_ASPECTS][NUM_SIGNAL_LEDS];
  // if a servo is configured for this signal (this is the usual case for form signals), this value represents the index of the servo in the SWITCHPORT_PIN array.
  // -1: no servo configured for this signal
  int servoIndex[NUM_SIGNAL_SERVOS];
  // the desired servo angle for the aspect (for form signals)
  int aspectServoAngle[NUM_SIGNAL_SERVOS][NUM_SIGNAL_ASPECTS];
} signalConfiguration[NUM_SIGNALS] = {};


// LEVEL CROSSING CONFIGURATION

// General switch for level crossing (false = no level crossing connected; true = level crossing connected)
const bool LEVEL_CROSSING_CONNECTED = false;

// Number of boom barrier servos configured for the level crossing
const int LC_NUM_BOOM_BARRIERS = 4;

// Number of signals configured for the level crossing
const int LC_NUM_LEDS = 4;

// Number of level crossing sensors
const int LC_NUM_SENSORS = 4;

// Number of tracks leading over the level crossing
const int LC_NUM_TRACKS = 2;

// Sensors (required for autonomous mode only)
struct LevelCrossingSensorConfiguration {
  // Sensor index (index within the sensorConfiguration array)
  int sensorIndex;
  // Track index of the sensor (0: track 1, 1: track 2 etc.)
  int track;
  // Purpose of the sensor (0: inbound / 1: outbound / 2: both)
  int purpose;
  // Orientation of the sensor (0: plus side / 1: minus side)
  int orientation;
};

struct LevelCrossingConfiguration {
  // Port configured in Rocrail for the level crossing
  // Attention: make sure the port does not conflict with a switch port!
  // -1 indicates "no port" = "does not accept rocrail commands". Useful for autonomous mode.
  int rocRailPort;

  // BOOM BARRIER CONFIGURATION
  // Servo ports (indices in the SWITCHPORT_PIN array)
  // servo 1 and 2 represents primary barriers, servo 3 and subsequent servos represents secondary barriers
  uint8_t servoIndex[LC_NUM_BOOM_BARRIERS];

  // Timings
  // Closing timespan for all boom barriers
  unsigned int bbClosingPeriod_ms;
  // Delay until primary boom barriers start closing
  unsigned int bbClosingDelayPrimary_ms;
  // Delay until secondary boom barriers start closing
  unsigned int bbClosingDelaySecondary_ms;
  // Opening timespan for all boom barriers
  unsigned int bbOpeningPeriod_ms;
  // Servo angles for "up" and "down" positions
  // Approximate closed (opened) angles for TrixBrix boom barrier servos to start with:
  // - If servo is connected directly to the ESP8266:
  // -- Primary booms: Right hand traffic: 0 (90), left hand traffic: 180 (90)
  // -- Secondary booms: Right hand traffic: 180 (90), left hand traffic: 0 (90)
  // - If servo is connected to PCA9685:
  // -- Primary booms: Right hand traffic: 30 (87), left hand traffic: 143 (87)
  // -- Secondary booms: Right hand traffic: 143 (87), left hand traffic: 30 (87)
  unsigned int bbAnglePrimaryUp;
  unsigned int bbAnglePrimaryDown;
  unsigned int bbAngleSecondaryUp;
  unsigned int bbAngleSecondaryDown;

  // FLASHING LIGHT (LED) CONFIGURATION
  // Signal ports (indices in the SIGNALPORT_PIN array)
  uint8_t ledIndex[LC_NUM_LEDS];
  // Signal flashing period in milliseconds (full cycle).
  unsigned int ledFlashingPeriod_ms;
  // Set to true to enable signal fading (brightens and fades lights gradually for enhanced realism)
  bool ledsFading;

  // VIRTUAL SENSOR CONFIGURATION
  // Virtual sensor for "booms closed" feedback event (index in the sensorConfiguration array). This virtual sensor is triggered after the boom barriers have closed.
  // Must be set to -1 to skip virtual "booms closed" sensor event
  int sensorIndexBoomsClosed;
  // Virtual sensor for "booms opened" feedback event (index in the sensorConfiguration array). This virtual sensor is triggered after the boom barriers have opened.
  // Must be set to -1 to skip virtual "booms opened" sensor event
  int sensorIndexBoomsOpened;

  // AUTONOMOUS MODE CONFIGURATION
  // Autonomous Mode enabled?
  bool autonomousModeEnabled;
  // Timeout after which axle counters are reset and the track is released (in milliseconds)
  unsigned int trackReleaseTimeout_ms;
  // Sensors
  LevelCrossingSensorConfiguration sensorConfiguration[LC_NUM_SENSORS];
} levelCrossingConfiguration = {};


// BASCULE BRIDGE CONFIGURATION

// General switch for bascule bridge (false = no bridge connected; true = bridge connected)
bool BASCULE_BRIDGE_CONNECTED = false;

// Number of bridge Leafs (equals number of bridge servos)
const int NUM_BASCULE_BRIDGE_LEAFS = 0;

struct BridgeLeafConfiguration {
  // Servo pin for bridge motor control
  int servoIndex;

  // Motor power settings for bridge operations (use negative values to reverse servo)
  int powerUp;  // Motor power for pulling the bridge up (0 .. 100)
  int powerUp2;  // Motor power for pulling the bridge up after the "bridge up" sensor has been triggered (0 .. 100)
  int powerDown;  // Motor power for letting the bridge down (0 .. 100)
  int powerDown2;  // Motor power for closing the bridge down after the "bridge down" sensor has been triggered (0 .. 100)

  // Sensor ports
  // local sensors that indicate "bridge leaf down" for each bridge leaf (index in the sensorConfiguration array)
  int sensorDown;
  // same for "up"
  int sensorUp;

  // Timings (in milli seconds)
  // Delay for opening the bridge leaf (in milliseconds)
  int delayOpen_ms;
  // Delay for closing the bridge leaf (in milliseconds)
  int delayClose_ms;
  // Maximum allowed time for opening the bridge from releasing the closing sensor until the opening sensor must have been triggered. After this time has passed, the bridge motor is stopped for safety reasons.
  int maxOpeningTime_ms;
  // Same for closing the bridge
  int maxClosingTime_ms;
  // Extra time after the "bridge up" sensor has been triggered until the bridge motor is stopped.
  int extraTimeAfterOpened_ms;
  // Extra time after the "bridge down" sensor has been triggered until the bridge motor is stopped.
  int extraTimeAfterClosed_ms;
};

struct BridgeConfiguration {
  // Port configured for the bascule bridge in Rocrail
  int rocRailPort;

  // Signal ports (set to -1 for "not connected")
  int signalRiverStop;  // signal port that is activated when bridge is not in the "up" position (index in the SIGNALPORT_PIN array)
  int signalRiverPrep;  // signal port that is activated in addition to the "stop" port when bridge is opening (index in the SIGNALPORT_PIN array)
  int signalRiverGo;  // signal port that is activated when bridge has reached the "up" position (index in the SIGNALPORT_PIN array)
  int signalBlinkLight;  // signal port for a blinking light that indicates opening/closing action (index in the SIGNALPORT_PIN array)

  // virtual sensor that indiciates "bridge fully down" (index in the sensorConfiguration array). This virtual sensor is triggered after the "extra time after closed".
  // Must be set to -1 to skip virtual "bridge fully down" sensor events
  int sensorFullyDown;
  // same for "up"
  int sensorFullyUp;

  // Bridge leafs
  BridgeLeafConfiguration leafConfiguration[NUM_BASCULE_BRIDGE_LEAFS];
} bridgeConfiguration = {};


// SPEEDOMETER CONFIGURATION

// General switch for speedometer (false = no speedometer connected; true = speedometer connected)
bool SPEEDOMETER_CONNECTED = true;

enum struct SpeedometerSpeedUnit
{
  STUDS_PER_SECOND,
  MILLIMETERS_PER_SECOND,
  KILOMETER_PER_HOUR,
  MILES_PER_HOUR
};

enum struct SpeedometerLengthUnit
{
  NO_INDICATION,
  STUDS,
  MILLIMETERS,
  CENTIMETERS,
  METERS
};

struct SpeedometerConfiguration {
  // speed unit (for display only - internally, mm/s is used)
  SpeedometerSpeedUnit speedUnit;
  // length unit (for display only - internally, mm is used)
  SpeedometerLengthUnit lengthUnit;
  // Indexes in the sensorConfiguration array used for measuring speed
  // There are always two sensors. The sensors MUST have the indices 0 and 1!
  // Only local sensors are supported
  int sensorIndex[2];
  // Distance between the sensors in mm
  // SM_DISTANCE must be larger than the distance between the magnets on the train
  float distance;
  // Timeout to reset the speedometer when nothing is happening anymore (in ms)
  unsigned int timeOut;
  // Minimum time between two measurements (in ms)
  unsigned int timeBetweenMeasurements;
  // Minimum time to display the speed on the display before switching to a "Screensaver" (in ms)
  unsigned int timeToShowResults;
} speedometerConfiguration =
{
  .speedUnit = SpeedometerSpeedUnit::STUDS_PER_SECOND,
  .lengthUnit = SpeedometerLengthUnit::STUDS,
  .sensorIndex = {0, 1},  // must be 0 and 1!
  .distance = 792,  // 99 studs
  .timeOut = 3000,
  .timeBetweenMeasurements = 3000,
  .timeToShowResults = 5000
};


// ****************
// Network settings
// ****************

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT true


// ***************
// Syslog settings
// ***************
// Syslog application name
const char* SYSLOG_APP_NAME = "MLC";
