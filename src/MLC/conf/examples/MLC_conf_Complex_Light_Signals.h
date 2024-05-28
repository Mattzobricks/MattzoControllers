// *********************
// COPYRIGHT AND LICENSE
// *********************

// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte

// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



// *****************
// PURPOSE AND USAGE
// *****************

// MattzoLayoutController (MLC) example configuration file
// Usage: copy it onto the conf/my/controller_config.h file and adapted it to your needs
// General documentation: https://www.mattzobricks.com
// Parameter documentation: MLC/include/MLC_types.h

// This configuration contains the configuration for the MLC mega.
// It serves:
// - 16 switches
// - 1 complex H/V light main signal
// - 1 H/V light distant signal
// - 16 sensors.



// *******************************
// CONTROLLER WIRING CONFIGURATION
// *******************************

// PCA9685 WIRING CONFIGURATION

// Infos for output port expander PCA9685
// Usage:
// - If a USE_PCA9685 port expander is connected to your ESP8266, set USE_PCA9685 to true. Else, it must be set to false.
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
// - Board 1 has the address 0x40, Board 2 0x41 etc.
// Servos:
// - Just connect the servos as designed. Servos connected to PCA9685 must be mapped in SWITCHPORT_PIN_PCA9685.
// Signals:
// - Connecting TrixBrix signals to the PCA9685 is somewhat tricky, because of the "common anode" (common plus terminal) of the signal LEDs.
// - The solution is to connect the middle wire (plus) to V3V of the ESP8266 (NOT the plus pins of the PCA9685 ports), and the outer wires to the PCA9685 pins.
// - It is important to remember the correct way of setting a pin on the PCA9685 to:
// -- fully on: pwm.setPWM(port, 4096, 0);
// -- fully off: pwm.setPWM(port, 0, 4096);
// - Generally, it is better practice to connect light signals via MCP23017 port extenders
// Additional reference: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all

// PCA9685 port expander used?
#define USE_PCA9685 true

// PCA9685 OE pin supported?
#define PCA9685_OE_PIN_INSTALLED true
uint8_t PCA9685_OE_PIN = D7;

// Number of chained PCA9685 port extenders
#define NUM_PCA9685s 1



// MCP23017 WIRING CONFIGURATION

// Infos for I/O port expander MCP23017
// Usage:
// - If a MCP23017 I/O port expander is connected to your ESP8266, set USE_MCP23017 to true.
// Wiring:
// - SCL and SDA are usually connected to pins D1 (clock) and D2 (data) of the ESP8266.
// - VCC is sourced from V3V of the ESP8266.
// - GND is connected to GND of the ESP8266. Connecting GND is mandatory!
// - RESET is connected with an 10K resistor to VCC
// - Address ports A0, A1 and A2 according to the desired address (0x20, 0x21, ...). All connected to GND means address 0x20.
// Ports:
// - The ports of the are numbered as follows:
// - A0..A7: 0..7
// - B0..B7: 8..15
// Chaining:
// - Both MCP23017 and the firmware support chaining.
// - Board 1 has the address 0x20, board 2 0x21 etc.
// - Up to 8 boards can be connected.
// Sensors:
// - Connecting sensors to the MCP23017 is simple.
// - Just connect one of of the cable pair to GND, the other one to one of the ports of the MCP23017.

// MCP23017 port expander used?
#define USE_MCP23017 true

// Number of chained MCP23017 port extenders
#define NUM_MCP23017s 2



// SERVO WIRING CONFIGURATION

// Servos are used for motorizing switches and form signals

// Number of servos
#define NUM_SERVOS 16

TServoConfiguration servoConfiguration[NUM_SERVOS] =
{
    {
        .pin = 0,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 1,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 2,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 3,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 4,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 5,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 6,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 7,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 8,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 9,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {   
        .pin = 10,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 11,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {   
        .pin = 12,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 13,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 14,
        .pinType = 0x40,
        .detachAfterUsage = true
    },
    {
        .pin = 15,
        .pinType = 0x40,
        .detachAfterUsage = true}
};



// LED WIRING CONFIGURATION

// LEDs are used in signals, level crossing lights or bascule bridge lights
// As an example, 2 LEDs are required for a light signal with 2 aspects

// Number of LEDs
#define NUM_LEDS 16

TLEDConfiguration ledConfiguration[NUM_LEDS] =
{
    {
        .pin = 0,
        .pinType = 0x21
    },
    {
        .pin = 8,
        .pinType = 0x21
    },
    {
        .pin = 1,
        .pinType = 0x21
    },
    {
        .pin = 9,
        .pinType = 0x21
    },
    {
        .pin = 2,
        .pinType = 0x21
    },
    {
        .pin = 10,
        .pinType = 0x21
    },
    {
        .pin = 3,
        .pinType = 0x21
    },
    {
        .pin = 11,
        .pinType = 0x21
    },
    {
        .pin = 4,
        .pinType = 0x21
    },
    {
        .pin = 12,
        .pinType = 0x21
    },
    {
        .pin = 5,
        .pinType = 0x21
    },
    {
        .pin = 13,
        .pinType = 0x21
    },
    {
        .pin = 6,
        .pinType = 0x21
    },
    {
        .pin = 14,
        .pinType = 0x21
    },
    {
        .pin = 7,
        .pinType = 0x21
    },
    {
        .pin = 15,
        .pinType = 0x21
    }
};



// SENSOR WIRING CONFIGURATION

// Sensors are generally used to indicate that a train have reached a specific position on the layout
// Special forms are remote and virtual sensors (see below)

// Number of sensors connected or connectable to the controller
#define NUM_SENSORS 16

// A special form of a sensor is the "remote sensor"
// Remote sensors are not electrically connected to this controller, they are triggered via Rocrail commands.
// Remote sensors can be used for level crossings in Autonomous Mode.
// Set REMOTE_SENSORS_ENABLED to true to generally enable remote sensors.
// If you do not control a level crossing in Autonomous Mode with this controller, set to false!
#define REMOTE_SENSORS_ENABLED false

TSensorConfiguration sensorConfiguration[NUM_SENSORS] =
{
    {
        .pin = 0,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1},
    {
        .pin = 1,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 2,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 3,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 4,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 5,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 6,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 7,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 8,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 9,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 10,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 11,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 12,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 13,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 14,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    },
    {
        .pin = 15,
        .pinType = MCP23017_SENSOR_PIN_TYPE,
        .remoteMattzoControllerId = -1
    }
};



// STATUS LED WIRING CONFIGURATION

// Digital output pin to monitor controller operation (typically a LED)
// Set to false if no status LED is installed
const bool STATUS_LED_PIN_INSTALLED = true;
// If installed, the pin controlling the status LED
const uint8_t STATUS_LED_PIN = D4;
// If installed, set to true to flip high/low state of the status led pin
const bool STATUS_LED_REVERSE = true;
// Power level of the status LED (0..1023)
// Recommended max. power levels: white: 800, blue: 600, green: 500, yellow: 350, red: 300
const int STATUS_LED_POWER = 300;



// ****************************
// LOGICAL OBJECT CONFIGURATION
// ****************************

// SWITCH CONFIGURATION

// Number of switches
#define NUM_SWITCHES 16

TSwitchConfiguration switchConfiguration[NUM_SWITCHES] =
{
    {
        .rocRailPort = 1,
        .servoIndex = 0,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 2,
        .servoIndex = 1,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 3,
        .servoIndex = 2,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 4,
        .servoIndex = 3,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 5,
        .servoIndex = 4,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 6,
        .servoIndex = 5,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 7,
        .servoIndex = 6,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 8,
        .servoIndex = 7,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 9,
        .servoIndex = 8,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 10,
        .servoIndex = 9,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 11,
        .servoIndex = 10,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 12,
        .servoIndex = 11,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 13,
        .servoIndex = 12,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 14,
        .servoIndex = 13,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 15,
        .servoIndex = 14,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    },
    {
        .rocRailPort = 16,
        .servoIndex = 15,
        .servo2Index = -1,
        .servo2Reverse = false,
        .triggerSensors = false,
        .sensorIndex = {-1, -1}
    }
};



// SIGNAL CONFIGURATION

// Number of signals
#define NUM_SIGNALS 2
// Maximum number of signal aspects (e.g. 2 for red/green, 3 for red/green/yellow etc.)
#define NUM_SIGNAL_ASPECTS 5
// Number of signal LEDs (usually equal to NUM_SIGNAL_ASPECTS)
#define NUM_SIGNAL_LEDS 6
// Maximum number of servos for form signals (e.g. one for the primary and another one for the secondary semaphore)
// If no form signals are used, just set to 0
#define NUM_SIGNAL_SERVOS 0

TSignalConfiguration signalConfiguration[NUM_SIGNALS] =
{
    // signal 0: complex H/V light main signal
    {
        .signalRocrailPort = 1,
        .aspectRocrailPort = {1, 2, 3, 4, 0}, // irrelevant, because this signal has control type "aspect numbers"
        .aspectLEDPort = {0, 1, 3, 2, 5, 4},  // green, red left, red right, yellow, white diagonal (Sh1), white triangle (Zs1)
        .aspectLEDMapping = {
            {LED_OFF  , LED_ON   , LED_ON   , LED_OFF  , LED_OFF  , LED_OFF  }, // Hp00
            {LED_ON   , LED_OFF  , LED_OFF  , LED_OFF  , LED_OFF  , LED_OFF  }, // Hp1
            {LED_ON   , LED_OFF  , LED_OFF  , LED_ON   , LED_OFF  , LED_OFF  }, // Hp2
            {LED_OFF  , LED_ON   , LED_OFF  , LED_OFF  , LED_ON   , LED_OFF  }, // Hp0+Sh1
            {LED_OFF  , LED_ON   , LED_ON   , LED_OFF  , LED_OFF  , LED_ON   }, // Hp00+Zs1
        },
        .servoIndex = {},
        .aspectServoAngle = {},
        .overshootSensorIndex = -1
    },
    // signal 1: H/V light distant signal
    {
        .signalRocrailPort = -1,
        .aspectRocrailPort = {9, 10, 11, -1, -1}, // irrelevant, because this signal has control type "aspect numbers"
        .aspectLEDPort = {8, 9, 10, 11, 12, -1},  // upper yellow, lower yellow, upper green, lower green, limited distance white
        .aspectLEDMapping = {
            {LED_ON   , LED_ON   , LED_OFF  , LED_OFF  , LED_ON   , LED_NOP  }, // Vr0
            {LED_OFF  , LED_OFF  , LED_ON   , LED_ON   , LED_ON   , LED_NOP  }, // Vr1
            {LED_OFF  , LED_ON   , LED_ON   , LED_OFF  , LED_ON   , LED_NOP  }, // Vr2
            {LED_NOP  , LED_NOP  , LED_NOP  , LED_NOP  , LED_NOP  , LED_NOP  }, // unused
            {LED_NOP  , LED_NOP  , LED_NOP  , LED_NOP  , LED_NOP  , LED_NOP  }, // unused
        },
        .servoIndex = {},
        .aspectServoAngle = {},
        .overshootSensorIndex = -1
    }
};


// LEVEL CROSSING CONFIGURATION

// General switch for level crossing (false = no level crossing connected; true = level crossing connected)
#define LEVEL_CROSSING_CONNECTED false

// Number of boom barrier servos configured for the level crossing
#define LC_NUM_BOOM_BARRIERS 4

// Number of signals configured for the level crossing
#define LC_NUM_LEDS 4

// Number of level crossing sensors
#define LC_NUM_SENSORS 4

// Number of tracks leading over the level crossing
#define LC_NUM_TRACKS 2

TLevelCrossingConfiguration levelCrossingConfiguration = {};



// BASCULE BRIDGE CONFIGURATION

// General switch for bascule bridge (false = no bridge connected; true = bridge connected)
#define BASCULE_BRIDGE_CONNECTED false

// Number of bridge Leafs (equals number of bridge servos)
#define NUM_BASCULE_BRIDGE_LEAFS 0

TBridgeConfiguration bridgeConfiguration = {};



// SPEEDOMETER CONFIGURATION

// General switch for speedometer (false = no speedometer connected; true = speedometer connected)
#define SPEEDOMETER_CONNECTED false

TSpeedometerConfiguration speedometerConfiguration = {};



// ****************
// NETWORK SETTINGS
// ****************

// Trigger emergency brake upon disconnect
const bool TRIGGER_EBREAK_UPON_DISCONNECT = true;

// WiFi Hostname
// Allowed characters: a-z, A-Z, 0-9. From 2nd character, hyphens ("-") may also be used.
const char *MC_HOSTNAME = "MLC-MEGA-COMPLEX-LIGHT-SIGNAL";

// Syslog application name
const char *SYSLOG_APP_NAME = "MLC-MEGA-COMPLEX-LIGHT-SIGNAL";
