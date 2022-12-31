#pragma once
#include "Arduino.h"

typedef struct {
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
} TServoConfiguration;

typedef struct {
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
} TLEDConfiguration;

// Constants for type of digital input pins for sensors
// 0   : local sensor on the ESP-8266 (D0 .. D8)
// 0x10: remote sensor, triggered via Rocrail message (REMOTE_SENSOR_PIN_TYPE)
// 0x11: virtual sensor, triggered when a switch has been thrown, or a level crossing or bascule bridge has reached its fully open or closed position.
// 0x20: local sensor, connected to a port on the 1st MCP23017
// 0x21: local sensor, connected to a port on the 2nd MCP23017
// 0x22: local sensor, connected to a port on the 3rd MCP23017 etc.
#define LOCAL_SENSOR_PIN_TYPE 0
#define REMOTE_SENSOR_PIN_TYPE 0x10
#define VIRTUAL_SENSOR_PIN_TYPE 0x11
#define MCP23017_SENSOR_PIN_TYPE 0x20

typedef struct {
    // Digital input PINs for hall, reed or other digital sensors (pins like D0, D1 etc. for ESP-8266 I/O pins, numbers like 0, 1 etc. for pins of the MCP23017)
    // If sensor is a remote sensor, enter the "Address" of the sensor in Rocrail.
    // If sensor is a virtual sensor, the value has no meaning (set to -1 by convention).
    int8_t pin;

    // Type of digital input pins for sensors
    uint8_t pinType;

    // If sensor is a remote sensor, the MattzoControllerId of the MattzoController to which the sensor is connected must be entered into this array.
    // If sensor is local or virtual, the value has no meaning (set to -1 by convention)
    int remoteMattzoControllerId;
} TSensorConfiguration;

typedef struct SwitchConfiguration {
    int rocRailPort;
    int servoIndex;

    // servo2Index: servo index for second servo (required for TrixBrix double slip switches). If unused, set to -1.
    // servo2Reverse: set to true if second servo shall be reversed
    int servo2Index;
    bool servo2Reverse;

    // feedback sensors
    // set triggerSensors to true if used
    // The first value in the sensorIndex is the index of the virtual sensor in the sensorConfiguration array for the "straight" sensor, the second is for "turnout".
    // Both referenced sensors must be virtual sensors.
    bool triggerSensors;
    int sensorIndex[2];
} TSwitchConfiguration;

// Maximum number of signal aspects (e.g. red, green, yellow)
#define MAX_NUM_SIGNAL_ASPECTS 8
// Number of signal LEDs (usually equal to NUM_SIGNAL_ASPECTS)
#define MAX_NUM_SIGNAL_LEDS 8
// Maximum number of servos for form signals (e.g. one for the primary and another one for the secondary semaphore)
// If no form signals are used, just set to 0
#define MAX_NUM_SIGNAL_SERVOS 2

typedef struct {
    // the port configured in Rocrail for an aspect
    // 0: aspect not supported by this signal
    int aspectRocrailPort[MAX_NUM_SIGNAL_ASPECTS];
    // if a LED is configured for this aspect (this is the usual case for light signals), this value represents the index of the LED in the SIGNALPORT_PIN array.
    // -1: no LED configured for this aspect
    int aspectLEDPort[MAX_NUM_SIGNAL_LEDS];
    // mappings between aspects and LEDs (often a diagonal matrix)
    // true: LED is mapped for this aspect
    bool aspectLEDMapping[MAX_NUM_SIGNAL_ASPECTS][MAX_NUM_SIGNAL_LEDS];
    // if a servo is configured for this signal (this is the usual case for form signals), this value represents the index of the servo in the SWITCHPORT_PIN array.
    // -1: no servo configured for this signal
    int servoIndex[MAX_NUM_SIGNAL_SERVOS];
    // the desired servo angle for the aspect (for form signals)
    int aspectServoAngle[MAX_NUM_SIGNAL_SERVOS][MAX_NUM_SIGNAL_ASPECTS];
} TSignalConfiguration;

// Sensors (required for autonomous mode only)
typedef struct {
    // Sensor index (index within the sensorConfiguration array)
    int sensorIndex;
    // Track index of the sensor (0: track 1, 1: track 2 etc.)
    int track;
    // Purpose of the sensor (0: inbound / 1: outbound / 2: both)
    int purpose;
    // Orientation of the sensor (0: plus side / 1: minus side)
    int orientation;
} TLevelCrossingSensorConfiguration;

// Number of boom barrier servos configured for the level crossing
#define MAX_LC_NUM_BOOM_BARRIERS 4

// Number of signals configured for the level crossing
#define MAX_LC_NUM_LEDS 8

// Number of level crossing sensors
#define MAX_LC_NUM_SENSORS 8

#define MAX_LC_NUM_TRACKS 4

typedef struct {
    // Port configured in Rocrail for the level crossing
    // Attention: make sure the port does not conflict with a switch port!
    // -1 indicates "no port" = "does not accept rocrail commands". Useful for autonomous mode.
    int rocRailPort;

    // BOOM BARRIER CONFIGURATION
    // Servo ports (indices in the SWITCHPORT_PIN array)
    // servo 1 and 2 represents primary barriers, servo 3 and subsequent servos represents secondary barriers
    uint8_t servoIndex[MAX_LC_NUM_BOOM_BARRIERS];

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
    uint8_t ledIndex[MAX_LC_NUM_LEDS];
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
    TLevelCrossingSensorConfiguration sensorConfiguration[MAX_LC_NUM_SENSORS];
} TLevelCrossingConfiguration;

typedef struct {
    // Servo pin for bridge motor control
    int servoIndex;

    // Motor power settings for bridge operations (use negative values to reverse servo)
    int powerUp;    // Motor power for pulling the bridge up (0 .. 100)
    int powerUp2;   // Motor power for pulling the bridge up after the "bridge up" sensor has been triggered (0 .. 100)
    int powerDown;  // Motor power for letting the bridge down (0 .. 100)
    int powerDown2; // Motor power for closing the bridge down after the "bridge down" sensor has been triggered (0 .. 100)

    // Sensor ports
    // local sensors that indicate "bridge leaf down" for each bridge leaf (index in the sensorConfiguration array)
    int sensorDown;
    // same for "up"
    int sensorUp;

    // Timings (in milli seconds)
    // Delay for opening the bridge leaf (in milliseconds)
    long unsigned int delayOpen_ms;
    // Delay for closing the bridge leaf (in milliseconds)
    long unsigned int delayClose_ms;
    // Maximum allowed time for opening the bridge from releasing the closing sensor until the opening sensor must have been triggered. After this time has passed, the bridge motor is stopped for safety reasons.
    long unsigned int maxOpeningTime_ms;
    // Same for closing the bridge
    long unsigned int maxClosingTime_ms;
    // Extra time after the "bridge up" sensor has been triggered until the bridge motor is stopped.
    long unsigned int extraTimeAfterOpened_ms;
    // Extra time after the "bridge down" sensor has been triggered until the bridge motor is stopped.
    long unsigned int extraTimeAfterClosed_ms;
} TBridgeLeafConfiguration;

#define MAX_NUM_BASCULE_BRIDGE_LEAFS 4

typedef struct {
    // Port configured for the bascule bridge in Rocrail
    int rocRailPort;

    // Signal ports (set to -1 for "not connected")
    int signalRiverStop;  // signal port that is activated when bridge is not in the "up" position (index in the SIGNALPORT_PIN array)
    int signalRiverPrep;  // signal port that is activated in addition to the "stop" port when bridge is opening (index in the SIGNALPORT_PIN array)
    int signalRiverGo;    // signal port that is activated when bridge has reached the "up" position (index in the SIGNALPORT_PIN array)
    int signalBlinkLight; // signal port for a blinking light that indicates opening/closing action (index in the SIGNALPORT_PIN array)

    // virtual sensor that indiciates "bridge fully down" (index in the sensorConfiguration array). This virtual sensor is triggered after the "extra time after closed".
    // Must be set to -1 to skip virtual "bridge fully down" sensor events
    int sensorFullyDown;
    // same for "up"
    int sensorFullyUp;

    // Bridge leafs
    TBridgeLeafConfiguration leafConfiguration[MAX_NUM_BASCULE_BRIDGE_LEAFS];
} TBridgeConfiguration;

enum struct SpeedometerSpeedUnit {
    STUDS_PER_SECOND,
    MILLIMETERS_PER_SECOND,
    KILOMETER_PER_HOUR,
    MILES_PER_HOUR
};

enum struct SpeedometerLengthUnit {
    NO_INDICATION,
    STUDS,
    MILLIMETERS,
    CENTIMETERS,
    METERS
};

typedef struct {
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
} TSpeedometerConfiguration;