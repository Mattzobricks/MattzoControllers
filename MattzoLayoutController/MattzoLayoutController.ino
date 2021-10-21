// MattzoSwitchController Firmware
// Author: Dr. Matthias Runte
// Copyright 2020, 2021 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define MATTZO_CONTROLLER_TYPE "MattzoLayoutController"
#include <ESP8266WiFi.h>                          // WiFi library for ESP-8266
#include <Servo.h>                                // Servo library
#include "MattzoLayoutController_Configuration.h" // this file should be placed in the same folder
#include "MattzoController_Library.h"             // this file needs to be placed in the Arduino library folder

#if USE_PCA9685
#include <Wire.h>                                 // Built-in library for I2C
#endif

// SERVO VARIABLES AND CONSTANTS

// Create servo objects to control servos
Servo servo[NUM_SWITCHPORTS];

// Create PWM Servo Driver object (for PCA9685)
#if USE_PCA9685
#include <Adafruit_PWMServoDriver.h>              // Adafruit PWM Servo Driver Library for PCA9685 port expander. Tested with version 2.4.0.
Adafruit_PWMServoDriver pca9685[NUM_PCA9685s];
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#endif

#if USE_MCP23017
#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcp23017[NUM_MCP23017s];
#endif

#if USE_U8G2
#include <U8g2lib.h>                              // Ardunio library for displays
#include <Wire.h>                                 // Built-in library for I2C
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // in arduino ide pleas look at "File" - "Examples" - "U8g2" in the menu for examples
#endif


// Default values for TrixBrix switches (in case servo angles are not transmitted)
const int SERVO_MIN_ALLOWED = 50;   // minimum accepted servo angle from Rocrail. Anything below this value is treated as misconfiguration and is neglected and reset to SERVO_MIN.
const int SERVO_MIN = 75;           // a good first guess for the minimum angle of TrixBrix servos is 70
const int SERVO_START = 80;         // position after boot-up. For TrixBrix servos, this is more or less the middle position
const int SERVO_MAX = 85;           // a good first guess for the maximum angle of TrixBrix servos is 90
const int SERVO_MAX_ALLOWED = 120;  // maximum accepted servo angle from Rocrail. Anything above this value is treated as misconfiguration and is neglected and reset to SERVO_MAX.

// Delay between two switch operations
const int SWITCH_DELAY = 200;

// Time after servo operation until servo power is switched off (in milliseconds; 3000 = 3 sec.)
// Presently only supported when using PCA9685
const int SERVOSLEEPMODEAFTER_MS = 3000;

// time when servos will go to sleep mode
bool servoSleepMode = false;
unsigned long servoSleepModeFrom_ms = 0;


// SENSOR VARIABLES AND CONSTANTS

// Time in milliseconds until release event is reported after sensor has lost contact
const int SENSOR_RELEASE_TICKS = 100;
bool sensorState[NUM_SENSORS];
int sensorTriggerState[NUM_SENSORS];
unsigned long lastSensorContact_ms[NUM_SENSORS];


// LEVEL CROSSING VARIABLES AND CONSTANTS
enum struct LevelCrossingStatus
{
  OPEN = 0x1,
  CLOSED = 0x2,
};

struct LevelCrossing {
  LevelCrossingStatus levelCrossingStatus = LevelCrossingStatus::OPEN;
  unsigned long lastStatusChangeTime_ms = 0;

  bool boomBarrierActionInProgress = true;
  float servoAnglePrimaryBooms = LC_BOOM_BARRIER_ANGLE_PRIMARY_UP;
  float servoAngleSecondaryBooms = LC_BOOM_BARRIER_ANGLE_SECONDARY_UP;
  float servoAngleIncrementPerMS = 0;
  float servoTargetAnglePrimaryBooms = LC_BOOM_BARRIER_ANGLE_PRIMARY_UP;
  float servoTargetAngleSecondaryBooms = LC_BOOM_BARRIER_ANGLE_SECONDARY_UP;
  unsigned long lastBoomBarrierTick_ms = 0;

  unsigned int sensorEventCounter[LC_NUM_TRACKS][2][2];
  bool trackOccupied[LC_NUM_TRACKS];
  unsigned long trackOccupiedTimeout_ms[LC_NUM_TRACKS];
} levelCrossing;


// BASCULE BRIDGE VARIABLES AND CONSTANTS
enum struct BridgeStatus
{
  CLOSED,
  OPENING,
  OPENED,
  CLOSING,
  UNDEFINED,
  ERRoR
};

enum struct BridgeLeafStatus
{
  CLOSED,
  OPENING0,
  OPENING1,
  OPENING2,
  OPENING3,
  OPENED,
  CLOSING0,
  CLOSING1,
  CLOSING2,
  CLOSING3,
  UNDEFINED,
  ERRoR
};

enum struct BridgeCommand
{
  UP = 0,
  DOWN = 1,
  NONE = 2
};

struct BridgeLeaf {
  Servo bridgeLeafServo;
  BridgeLeafStatus leafStatus = BridgeLeafStatus::UNDEFINED;
  unsigned long leafTimer;
};

struct Bridge {
  BridgeStatus bridgeStatus = BridgeStatus::UNDEFINED;
  BridgeCommand bridgeCommand = BridgeCommand::NONE;
  BridgeLeaf bridgeLeaf[NUM_BASCULE_BRIDGE_LEAFS];
} bridge;


// SPEEDOMETER VARIABLES AND CONSTANTS
struct Speedometer {
  bool occupied = false;
  SpeedometerLengthUnit lengthUnit = SM_LENGTHUNIT;
  SpeedometerSpeedUnit speedUnit = SM_SPEEDUNIT;

  int startSensor;
  int endSensor;
  int wheelcounter[SM_NUM_SENSORS] = {-1, -1};

  float startTime[SM_MAX_VALUES];
  float endTime[SM_MAX_VALUES];
  float trainSpeed[SM_MAX_VALUES];
  float trainLength[SM_MAX_VALUES];

  unsigned long lastMeasurementEvent = -99999;
  unsigned long measurementDone = -99999;
  unsigned long animationDelay = -99999;

  float actualTrainSpeed  = 0;
  float actualTrainLength = 0;
} speedometer;



void setup() {
  Serial.begin(115200);

  // initialize PWM Servo Driver object (for PCA9685)
#if USE_PCA9685
  setupPCA9685();

  // Switch PCA9685 off
  if (PCA9685_OE_PIN_INSTALLED) {
    pinMode(PCA9685_OE_PIN, OUTPUT);
    setServoSleepMode(true);
  }
#endif

#if USE_MCP23017
  setupMCP23017();
#endif

#if USE_U8G2
  setupU8g2();
#endif

  // initialize servo pins and turn servos to start position
  for (int i = 0; i < NUM_SWITCHPORTS; i++) {
    if (SWITCHPORT_PIN_TYPE[i] == 0) {
      // servo connected directly to the controller
      servo[i].attach(SWITCHPORT_PIN[i]);
      servo[i].write(SERVO_START);
      delay(SWITCH_DELAY);
    }
    else if (SWITCHPORT_PIN_TYPE[i] >= 0x40) {
      // servo connected to PCA9685
      // no action required
    }
  }

  // stop bascule bridge motors
  if (BASCULE_BRIDGE_CONNECTED) {
    for (int l = 0; l < NUM_BASCULE_BRIDGE_LEAFS; l++) {
      setServoAngle(BASCULE_BRIDGE_SERVO_INDEX[l], 100);  // stop continuous servo of the bascule bridge leaf
    }
  }

  // initialize signal pins
  for (int i = 0; i < NUM_SIGNALPORTS; i++) {
    if (SIGNALPORT_PIN_TYPE[i] == 0) {
      // signal connected directly to the controller
      pinMode(SIGNALPORT_PIN[i], OUTPUT);
    }
    else if (SIGNALPORT_PIN_TYPE[i] >= 0x40) {
      // signal connected to PCA9685
      // no action required
    }
  }

  // initialize sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (SENSOR_PIN_TYPE[i] == LOCAL_SENSOR_PIN_TYPE) {
      // sensor connected directly to the controller
      pinMode(SENSOR_PIN[i], INPUT_PULLUP);
      sensorTriggerState[i] = (SENSOR_PIN[i] == D8) ? HIGH : LOW;
    }
    else if (SENSOR_PIN_TYPE[i] >= MCP23017_SENSOR_PIN_TYPE) {
      // sensor connected to MCP23017
#if USE_MCP23017
      int m = SENSOR_PIN_TYPE[i] - MCP23017_SENSOR_PIN_TYPE;  // index of the MCP23017
      mcp23017[m].pinMode(SENSOR_PIN[i], INPUT);
      mcp23017[m].pullUp(SENSOR_PIN[i], HIGH); // turn on a 100K pull-up resistor internally
      sensorTriggerState[i] = LOW;
#endif
    }
    sensorState[i] = false;
  }

  // load config from EEPROM, initialize Wifi, MQTT etc.
  setupMattzoController();
}

#if USE_PCA9685
void setupPCA9685() {
  // Initialize PWM Servo Driver object (for PCA9685)
  for (int p = 0; p < NUM_PCA9685s; p++) {
    pca9685[p] = Adafruit_PWMServoDriver(0x40 + p);
    pca9685[p].begin();
    pca9685[p].setOscillatorFrequency(27000000);
    pca9685[p].setPWMFreq(SERVO_FREQ);
    delay(10);
  }
}
#endif

#if USE_MCP23017
void setupMCP23017() {
  for (int m = 0; m < NUM_MCP23017s; m++) {
    mcp23017[m] = Adafruit_MCP23017();
    mcp23017[m].begin();
  }
}
#endif

#if USE_U8G2
void setupU8g2() {
  u8g2.begin();
}
#endif

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  msg[length] = '\0';

  mcLog2("Received MQTT message [" + String(topic) + "]: " + String(msg), LOG_DEBUG);

  XMLDocument xmlDocument;
  if(xmlDocument.Parse(msg)!= XML_SUCCESS){
    mcLog2("Error parsing XML: " + String(msg), LOG_ERR);
    return;
  }
  mcLog2("Parsing XML successful.", LOG_DEBUG);

  XMLElement *element;
  element = xmlDocument.FirstChildElement("sw");
  if (element != NULL) {
    // handle switch, level crossing or bascule bridge message
    mcLog2("<sw> node found.", LOG_DEBUG);

    // query addr1 attribute. This is the MattzoController id.
    // If this does not equal the mattzoControllerId of this controller, the message is disregarded.
    int rr_addr1 = 0;
    if (element->QueryIntAttribute("addr1", &rr_addr1) != XML_SUCCESS) {
      mcLog2("addr1 attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("addr1: " + String(rr_addr1), LOG_DEBUG);
    if (rr_addr1 != mattzoControllerId) {
      mcLog2("Message disgarded, as it is not for me (" + String(mattzoControllerId) + ")", LOG_DEBUG);
      return;
    }

    // query port1 attribute. This is port id of the port to which the switch is connected.
    // If the controller does not have such a port, the message is disregarded.
    int rr_port1 = 0;
    if (element->QueryIntAttribute("port1", &rr_port1) != XML_SUCCESS) {
      mcLog2("port1 attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("port1: " + String(rr_port1), LOG_DEBUG);
    if (BASCULE_BRIDGE_CONNECTED && (rr_port1 == BASCULE_BRIDGE_RR_PORT)) {
      mcLog2("This is a bascule bridge command.", LOG_DEBUG);
    }
    else if ((rr_port1 < 1 || rr_port1 > NUM_SWITCHPORTS) && (rr_port1 < 1001)) {
      mcLog2("Message disgarded, this controller does not have such a port.", LOG_ERR);
      return;
    }

    // query cmd attribute. This is the desired switch setting and can either be "turnout" or "straight".
    const char* rr_cmd = "-unknown-";
    if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
      mcLog2("cmd attribute not found or wrong type.", LOG_ERR);
      return;
    }
    mcLog2("cmd: " + String(rr_cmd), LOG_DEBUG);

    // query param1 attribute. This is the "straight" position of the switch servo motor.
    // defaults to SERVO_MIN
    int rr_param1 = SERVO_MIN;
    if (element->QueryIntAttribute("param1", &rr_param1) != XML_SUCCESS) {
      mcLog2("param1 attribute not found or wrong type. Using default value.", LOG_DEBUG);
    }
    if (rr_param1 < SERVO_MIN_ALLOWED || rr_param1 > SERVO_MAX_ALLOWED) {
      // Reset angle back to standard if angle is too small
      // User has obviously forgotten to configure servo angle in Rocrail properly
      // To protect the servo, the default value is used
      mcLog2("param1 attribute out of bounds. Using default value.", LOG_DEBUG);
      rr_param1 = SERVO_MIN;
    }
    mcLog2("param1: " + String(rr_param1), LOG_DEBUG);

    // query value1 attribute. This is the "turnout" position of the switch servo motor.
    // defaults to SERVO_MAX
    int rr_value1 = SERVO_MAX;
    if (element->QueryIntAttribute("value1", &rr_value1) != XML_SUCCESS) {
      mcLog2("value1 attribute not found or wrong type. Using default value.", LOG_DEBUG);
    }
    if (rr_value1 < SERVO_MIN_ALLOWED || rr_value1 > SERVO_MAX_ALLOWED) {
      // Reset angle back to standard if angle is too small
      // User has obviously forgotten to configure servo angle in Rocrail properly
      // To protect the servo, the default value is used
      mcLog2("value1 attribute out of bounds. Using default value.", LOG_DEBUG);
      rr_value1 = SERVO_MAX;
    }
    mcLog2("value1: " + String(rr_value1), LOG_DEBUG);

    // check command string and prepare servo angle
    // servo angle will only be used to flip a standard or one side of a triple switch - not for double slip switches!
    int switchCommand;
    if (strcmp(rr_cmd, "straight") == 0) {
      switchCommand = 1;
    }
    else if (strcmp(rr_cmd, "turnout") == 0) {
      switchCommand = 0;
    }
    else {
      mcLog2("Switch command unknown - message disregarded.", LOG_ERR);
      return;
    }

    // Check if port is used to control a level crossing
    if (LEVEL_CROSSING_CONNECTED && (rr_port1 == LEVEL_CROSSING_RR_PORT)) {
      levelCrossingCommand(switchCommand);
      return;
    }

    // Check if port is used to control a bascule bridge
    if (BASCULE_BRIDGE_CONNECTED && (rr_port1 == BASCULE_BRIDGE_RR_PORT)) {
      basculeBridgeCommand(switchCommand);
      return;
    }

    // Check if port is a logical switch port
    if (rr_port1 >= 1001) {
      // look for logical port in LOGICAL_SWITCHPORTS array
      bool logicalSwitchPortFound = false;
      for (int i = 0; i < NUM_LOGICAL_SWITCHPORTS; i++) {
        if (rr_port1 == LOGICAL_SWITCHPORTS[i]) {
          // logical switch port found
          logicalSwitchPortFound = true;

          int servoPort1 = LOGICAL_SWITCHPORT_MAPPINGS[i * 2];
          int servoPort2 = LOGICAL_SWITCHPORT_MAPPINGS[i * 2 + 1];
          int servoAngle1 = (switchCommand == 1) ? rr_param1 : rr_value1;
          int servoAngle2 = (switchCommand == 1 ^ LOGICAL_SWITCHPORT_REV_2ND_PORT[i]) ? rr_param1 : rr_value1;

          // Release virtual sensor on counterside
          sendSwitchSensorEvent(rr_port1, 1 - switchCommand, false);

          mcLog2("Turning servos on logical port " + String(rr_port1) + "...", LOG_DEBUG);
          setServoAngle(servoPort1, servoAngle1);
          setServoAngle(servoPort2, servoAngle2);

          // Trigger virtual sensor
          sendSwitchSensorEvent(rr_port1, switchCommand, true);
        }
      }

      if (!logicalSwitchPortFound) {
        mcLog2("Error: logical port " + String(rr_port1) + " unknown.", LOG_ERR);
        return;
      }
    }
    else {
      // Release virtual sensor on counterside
      sendSwitchSensorEvent(rr_port1, 1 - switchCommand, false);

      // Port was a plain, simple switch port
      setServoAngle(rr_port1 - 1, (switchCommand == 1) ? rr_param1 : rr_value1);

      // Trigger virtual sensor
      sendSwitchSensorEvent(rr_port1, switchCommand, true);
    }

    return;
    // end of switch command handling
  }

  element = xmlDocument.FirstChildElement("co");
  if (element != NULL) {
    // handle signal message
    mcLog2("<co> node found.", LOG_DEBUG);

    // query addr attribute. This is the MattzoController id.
    // If this does not equal the ControllerNo of this controller, the message is disregarded.
    int rr_addr = 0;
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog2("addr attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("addr: " + String(rr_addr), LOG_DEBUG);
    if (rr_addr != mattzoControllerId) {
      mcLog2("Message disgarded, as it is not for me (" + String(mattzoControllerId) + ")", LOG_ERR);
      return;
    }

    // query port attribute. This is port id of the port to which the signal is connected.
    // If the controller does not have such a port, the message is disregarded.
    int rr_port = 0;
    if (element->QueryIntAttribute("port", &rr_port) != XML_SUCCESS) {
      mcLog2("port attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("port: " + String(rr_port), LOG_DEBUG);
    if (rr_port < 1) {
      mcLog2("Message disgarded, as the signal port is below 1.", LOG_ERR);
      return;
    }

    // query cmd attribute. This is the desired signal setting and can either be "on" or "off".
    const char* rr_cmd = "-unknown-";
    if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
      mcLog2("cmd attribute not found or wrong type.", LOG_ERR);
      return;
    }
    mcLog2("cmd: " + String(rr_cmd), LOG_DEBUG);

    // parse signal command
    if (strcmp(rr_cmd, "on") == 0) {
      // only signal message with command 'on' will be processed
      handleSignalMessage(rr_port);
    }
    else if (strcmp(rr_cmd, "off") == 0) {
      // disregard signal messages with command 'off'
      mcLog2("Signal command 'off' received - message disregarded.", LOG_DEBUG);
      return;
    }
    else {
      mcLog2("Signal command " + String(rr_cmd) + " unknown - message disregarded.", LOG_DEBUG);
      return;
    }

    return;
    // end of signal handling
  }

  if (REMOTE_SENSORS_ENABLED) {
    element = xmlDocument.FirstChildElement("fb");
    if (element != NULL) {
      // handle feedback message. Used for remote sensors
      mcLog2("<fb> node found.", LOG_DEBUG);

      // query bus attribute. This MattzoControllerId to which the sensor is connected
      // If the bus attribute is not found, the message is discarded.
      int rr_bus = 0;
      if (element->QueryIntAttribute("bus", &rr_bus) != XML_SUCCESS) {
        mcLog2("bus attribute not found or wrong type. Message disregarded.", LOG_ERR);
        return;
      }
      mcLog2("bus: " + String(rr_bus), LOG_DEBUG);
      // If the received MattzoControllerId equals the Id of this controller, the message is discarded as it was originated by this controller in the first place.
      if (rr_bus == mattzoControllerId) {
        mcLog2("Message disregarded as it was originated by this controller.", LOG_DEBUG);
        return;
      }

      // query addr attribute. This is the number of the sensor on the remote controller.
      // If this does not equal the ControllerNo of this controller, the message is disregarded.
      int rr_addr = 0;
      if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
        mcLog2("addr attribute not found or wrong type. Message disregarded.", LOG_ERR);
        return;
      }

      // query state attribute. This is the sensor state and can either be "true" (triggered) or "false" (not triggered).
      const char* rr_state = "xXxXx";
      if (element->QueryStringAttribute("state", &rr_state) != XML_SUCCESS) {
        mcLog2("state attribute not found or wrong type.", LOG_ERR);
        return;
      }
      mcLog2("state: " + String(rr_state), LOG_DEBUG);
      bool sensorState = strcmp(rr_state, "true") == 0;

      // handle remote sensor event
      handleRemoteSensorEvent(rr_bus, rr_addr, sensorState);

      // end of remote sensor handling
    }
    return;
  }

  mcLog2("Unhandled message type. Message disregarded.", LOG_DEBUG);
}


// handles a signal mesage that was received from Rocrail
void handleSignalMessage(int rr_port) {
  // Look for the signal that uses rr_port
  // Edge case: multiple signals may share the same rr_port

  if (rr_port < 1) return;

  for (int s = 0; s < NUM_SIGNALS; s++) {
    for (int a = 0; a < NUM_SIGNAL_ASPECTS; a++) {
      if (signals[s].aspectRocrailPort[a] == rr_port) {
        // found the aspect that corresponds with rr_port
        // -> set aspect a for signal s

        // iterate through all configured LEDs for the signal and set it corresponding to the aspect LED matrix
        for (int l = 0; l < NUM_SIGNAL_ASPECTS; l++) {
          bool onOff = signals[s].aspectLEDMapping[a][l];
          mcLog2("Setting signal LED index " + String(l) + " of signal " + String(s) + " to " + (onOff ? "on" : "off"), LOG_INFO);
          setSignalLED(signals[s].aspectLEDPort[l], onOff);
        }

        // set the desired servo angle of the form signal
        for (int servo = 0; servo < NUM_SIGNAL_SERVOS; servo++) {
          // skip servo if servo pin < 0 (this means "not used")
          if (signals[s].servoIndex[servo] >= 0) {
            int servoAngle = signals[s].aspectServoAngle[servo][a];
            mcLog2("Turning servo index " + String(servo) + " of signal " + String(s) + " to " + String(servoAngle), LOG_INFO);
            setServoAngle(signals[s].servoIndex[servo], servoAngle);
          }
        }
      }
    }
  }

  ////
  //if (SIGNALPORT_PIN_TYPE[rr_port - 1] != 1) {
  //  // light signal => switch LED for this signal port on/off
  //  mcLog2("Setting signal LED index " + String(rr_port - 1) + " to " + signalCommand ? "on" : "off", LOG_DEBUG);
  //  setSignalLED(rr_port - 1, signalCommand);
  //}
  //else {
  //  // form signal => turn servo to the desired value
  //  // Parse servo angle
  //  int servoAngle;
  //  if (signalCommand) {
  //    // red aspect. Use "value" attribute (in the Rocrail interface, this parameter is labelled "Brightness")!
  //    if (element->QueryIntAttribute("value", &servoAngle) != XML_SUCCESS) {
  //      mcLog2("Error in form signal configuration: value attribute not found or wrong type.", LOG_ERR);
  //      return;
  //    }
  //  }
  //  else {
  //    // green aspect. Use "param" attribute (in the Rocrail interface, this parameter is labelled "Parameter")!
  //    if (element->QueryIntAttribute("param", &servoAngle) != XML_SUCCESS) {
  //      mcLog2("Error in form signal configuration: param attribute not found or wrong type.", LOG_ERR);
  //      return;
  //    }
  //  }
  //  setServoAngle(SIGNALPORT_PIN[rr_port - 1], servoAngle);
  //}

}


void sendSensorEvent2MQTT(int sensorPort, bool sensorState) {
  if (sensorPort < 0) {
    mcLog2("Sensor message skipped (sensorPort = " + String(sensorPort) + ")", LOG_DEBUG);
    return;
  }

  String sensorRocId = MATTZO_CONTROLLER_TYPE + String(mattzoControllerId) + "-" + String(sensorPort + 1);  // e.g. "MattzoController12345-3"
  String stateString = sensorState ? "true" : "false";

  // compile mqtt message. Parameters:
  //   id: Combination of sensor name and port (e.g. MattzoController12345-3). The reported port (the "logic" port) is 1 count higher than the internal port number in the sensor, e.g. port 2 in the sensor equals 3 in Rocrail)
  //   bus: controller number
  //   address: port number (internal port number plus 1)
  // both id or bus/address can be used in Rocrail. If id is used, it superseeds the combination of bus and address
  String mqttMessage = "<fb id=\"" + sensorRocId + "\" bus=\"" + String(mattzoControllerId) + "\" addr=\"" + String(sensorPort + 1) + "\" state=\"" + stateString + "\"/>";
  mcLog2("Sending MQTT message: " + mqttMessage, LOG_DEBUG);
  char mqttMessage_char[255];   // message is usually 61 chars, so 255 chars should be enough
  mqttMessage.toCharArray(mqttMessage_char, mqttMessage.length() + 1);
  mqttClient.publish("rocrail/service/client", mqttMessage_char);
}

void sendEmergencyBrake2MQTT(String emergencyBrakeReason) {
  String mqttMessage = "<sys cmd=\"ebreak\" reason=\"" + emergencyBrakeReason + "\"/>";
  mcLog2("Sending emergency brake message via MQTT: " + mqttMessage, LOG_ERR);
  char mqttMessage_char[255];   // message with reason "bridge open" is 44 chars, so 255 chars should be enough
  mqttMessage.toCharArray(mqttMessage_char, mqttMessage.length() + 1);
  mqttClient.publish("rocrail/service/client", mqttMessage_char);
}

// Switches LED on if one or more sensors has contact
// Switches LED off if no sensor has contact
void setLEDBySensorStates() {
  bool ledOnOff = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorState[i]) {
      statusLEDState = true;
      return;
    }
  }
  statusLEDState = false;
}

void monitorSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    // monitor local sensors
    if (SENSOR_PIN_TYPE[i] == LOCAL_SENSOR_PIN_TYPE || SENSOR_PIN_TYPE[i] >= MCP23017_SENSOR_PIN_TYPE) {
      int sensorValue;
      if (SENSOR_PIN_TYPE[i] == LOCAL_SENSOR_PIN_TYPE) {
        // sensor directly connected to ESP8266
        sensorValue = digitalRead(SENSOR_PIN[i]);
      }
      else if (SENSOR_PIN_TYPE[i] >= MCP23017_SENSOR_PIN_TYPE) {
        // sensor connected to MCP23017
  #if USE_MCP23017
        int m = SENSOR_PIN_TYPE[i] - MCP23017_SENSOR_PIN_TYPE;  // index of the MCP23017
        sensorValue = mcp23017[m].digitalRead(SENSOR_PIN[i]);
  #endif
      }
  
      if (sensorValue == sensorTriggerState[i]) {
        // Contact -> report contact immediately
        if (!sensorState[i]) {
          mcLog2("Sensor " + String(i) + " triggered.", LOG_INFO);
          sendSensorEvent2MQTT(i, true);
          sensorState[i] = true;
          handleSpeedometerSensorEvent(i);
          handleLevelCrossingSensorEvent(i);
        }
        lastSensorContact_ms[i] = millis();
      }
      else {
        // No contact for SENSOR_RELEASE_TICKS milliseconds -> report sensor has lost contact
        if (sensorState[i] && (millis() > lastSensorContact_ms[i] + SENSOR_RELEASE_TICKS)) {
          mcLog2("Sensor " + String(i) + " released.", LOG_INFO);
          sendSensorEvent2MQTT(i, false);
          sensorState[i] = false;
        }
      }
    }
  }

  setLEDBySensorStates();
}

// handle a remote sensor event.
// remote sensor events are used for level crossings in Autonomous Mode
void handleRemoteSensorEvent(int mcId, int sensorAddress, bool sensorState) {
  // find sensor in sensor array
  // if found, handle level crossing sensor event
  for (int s = 0; s < NUM_SENSORS; s++) {
    if (SENSOR_PIN_TYPE[s] == REMOTE_SENSOR_PIN_TYPE) {
      if (SENSOR_REMOTE_MATTZECONTROLLER_ID[s] == mcId) {
        if (SENSOR_PIN[s] == sensorAddress) {
          mcLog2("Remote sensor " + String(mcId) + ":" + String(sensorAddress) + " found.", LOG_DEBUG);
          if (sensorState) {
            handleLevelCrossingSensorEvent(s);
          }
          return;
        }
      }
    }
  }
  mcLog2("Remote sensor " + String(mcId) + ":" + String(sensorAddress) + " not found.", LOG_DEBUG);
}

// sets the servo arm to a desired angle
void setServoAngle(int servoIndex, int servoAngle) {
  // mcLog2("Turning servo index " + String(servoIndex) + " to angle " + String(servoAngle), LOG_DEBUG);
  if (servoIndex >= 0 && servoIndex < NUM_SWITCHPORTS) {
    if (SWITCHPORT_PIN_TYPE[servoIndex] == 0) {
      servo[servoIndex].write(servoAngle);
    }
#if USE_PCA9685
    else if (SWITCHPORT_PIN_TYPE[servoIndex] >= 0x40) {
      setServoSleepMode(false);
      pca9685[SWITCHPORT_PIN_TYPE[servoIndex] - 0x40].setPWM(SWITCHPORT_PIN[servoIndex], 0, mapAngle2PulseLength(servoAngle));
    }
#endif
    // delay(SWITCH_DELAY);
  }
  else {
    // this should not happen
    mcLog2("WARNING: servo index " + String(servoIndex) + " out of range!", LOG_ALERT);
  }
}

// converts a desired servo angle (0 .. 180 degrees) into a pwm pulse length (required for PCA9685)
int mapAngle2PulseLength(int angle) {
  const int PULSE_MIN = 0;
  const int PULSE_MAX = 600;
  return map(angle, 0, 180, PULSE_MIN, PULSE_MAX);
}

// Switches servo power supply on or off (presently supported for PCA9685 only)
void setServoSleepMode(bool onOff) {
  if (PCA9685_OE_PIN_INSTALLED) {
    if (servoSleepMode != onOff) {
      digitalWrite(PCA9685_OE_PIN, onOff ? HIGH : LOW);
      servoSleepMode = onOff;
      if (onOff) {
        mcLog("Servo power turned off.");
      } else {
        mcLog("Servo power turned on.");
      }
    }
    if (!onOff) {
      servoSleepModeFrom_ms = millis() + SERVOSLEEPMODEAFTER_MS;
    }
  }
}

// Checks if servos can be set to sleep mode (presently supported for PCA9685 only)
void checkEnableServoSleepMode() {
  if (millis() > servoSleepModeFrom_ms) {
    setServoSleepMode(true);
  }
}

// Releases or triggers a virtual switch sensor
void sendSwitchSensorEvent(int rocrailPort, int switchCommand, bool sensorState) {
  if (NUM_SWITCHPORT_SENSORPAIRS > 0) {
    if (switchCommand < 0 || switchCommand > 1) {
      // This should never happen
      mcLog2("sendSwitchSensorEvent() received switchCommand out of bounds: " + String(switchCommand), LOG_CRIT);
      return;
    }
  
    // Search for switch sensors
    for (int s = 0; s < NUM_SWITCHPORT_SENSORPAIRS; s++) {
      if (rocrailPort == SWITCHPORT_SENSORS[s][0]) {
        // Sensor for switchIndex found!
        int sensorIndex = SWITCHPORT_SENSORS[s][2 - switchCommand];
        mcLog2("Sending switch sensor event; rocrailPort=" + String(rocrailPort) + ", switchCommand=" + String(switchCommand) + ", sensorState=" + String(sensorState), LOG_DEBUG);
        sendSensorEvent2MQTT(sensorIndex, sensorState);
        return;
      }
    }
  
    mcLog2("No switch sensor found for rocrailPort=" + String(rocrailPort), LOG_DEBUG);
  }
}


// switches a signal on or off
void setSignalLED(int signalIndex, bool ledState) {
  if (signalIndex < 0)
    return;

  if (SIGNALPORT_PIN_TYPE[signalIndex] == 0) {
    digitalWrite(SIGNALPORT_PIN[signalIndex], ledState ? LOW : HIGH);
  }
#if USE_PCA9685
  else if (SIGNALPORT_PIN_TYPE[signalIndex] >= 0x40) {
    if (ledState) {
      // full bright
      pca9685[SIGNALPORT_PIN_TYPE[signalIndex] - 0x40].setPWM(SIGNALPORT_PIN[signalIndex], 4096, 0);
      // half bright (strongly dimmed)
      // pca9685[SIGNALPORT_PIN_TYPE[signalIndex] - 0x40].setPWM(SIGNALPORT_PIN[signalIndex], 0, 2048);
      // 3/4 bright (slightly dimmed)
      // pca9685[SIGNALPORT_PIN_TYPE[signalIndex] - 0x40].setPWM(SIGNALPORT_PIN[signalIndex], 0, 3072);
    }
    else {
      // off
      pca9685[SIGNALPORT_PIN_TYPE[signalIndex] - 0x40].setPWM(SIGNALPORT_PIN[signalIndex], 0, 4096);
    }
  }
#endif
}

// fades a signal. "brightness" is a value between 0 (off) and 1023 (full bright)
void fadeSignalLED(int signalIndex, int brightness) {
  if (signalIndex < 0)
    return;

  brightness = min(max(brightness, 0), 1023);
  
  if (SIGNALPORT_PIN_TYPE[signalIndex] == 0) {
    analogWrite(SIGNALPORT_PIN[signalIndex], 1023 - brightness);
  }
#if USE_PCA9685
  else if (SIGNALPORT_PIN_TYPE[signalIndex] >= 0x40) {
    if (brightness == 1023) {
      // full bright
      pca9685[SIGNALPORT_PIN_TYPE[signalIndex] - 0x40].setPWM(SIGNALPORT_PIN[signalIndex], 0, 4096);
    }
    else if (brightness == 0) {
      // off
      pca9685[SIGNALPORT_PIN_TYPE[signalIndex] - 0x40].setPWM(SIGNALPORT_PIN[signalIndex], 4096, 0);
    }
    else {
      // some other brightness value
      pca9685[SIGNALPORT_PIN_TYPE[signalIndex] - 0x40].setPWM(SIGNALPORT_PIN[signalIndex], 0, 4096 - brightness * 4);
    }
  }
#endif
}


// copy level crossing command to level crossing object
void levelCrossingCommand(int levelCrossingCommand) {
  if (levelCrossingCommand == 0) { // open
    if (levelCrossing.levelCrossingStatus != LevelCrossingStatus::OPEN) {
      // If level crossing operates in autonomous mode, check if a track is occupied
      if (!LC_AUTONOMOUS_MODE || !lcIsOccupied()) {
        levelCrossing.levelCrossingStatus = LevelCrossingStatus::OPEN;
        levelCrossing.servoTargetAnglePrimaryBooms = LC_BOOM_BARRIER_ANGLE_PRIMARY_UP;
        levelCrossing.servoTargetAngleSecondaryBooms = LC_BOOM_BARRIER_ANGLE_SECONDARY_UP;
        levelCrossing.servoAngleIncrementPerMS = (float)abs(LC_BOOM_BARRIER_ANGLE_PRIMARY_UP - LC_BOOM_BARRIER_ANGLE_PRIMARY_DOWN) / LC_BOOM_BARRIER_OPENING_PERIOD_MS;
        mcLog2("Level crossing command OPEN, servo increment " + String(levelCrossing.servoAngleIncrementPerMS * 1000) + " deg/s.", LOG_INFO);
        levelCrossing.lastStatusChangeTime_ms = millis();
        levelCrossing.boomBarrierActionInProgress = true;
        sendSensorEvent2MQTT(LEVEL_CROSSING_SENSOR_BOOMS_CLOSED, false);
      }
    }
  }
  else if (levelCrossingCommand == 1) { // closed
    if (levelCrossing.levelCrossingStatus != LevelCrossingStatus::CLOSED) {
      levelCrossing.levelCrossingStatus = LevelCrossingStatus::CLOSED;
      levelCrossing.servoTargetAnglePrimaryBooms = LC_BOOM_BARRIER_ANGLE_PRIMARY_DOWN;
      levelCrossing.servoTargetAngleSecondaryBooms = LC_BOOM_BARRIER_ANGLE_SECONDARY_DOWN;
      levelCrossing.servoAngleIncrementPerMS = (float)abs(LC_BOOM_BARRIER_ANGLE_PRIMARY_UP - LC_BOOM_BARRIER_ANGLE_PRIMARY_DOWN) / LC_BOOM_BARRIER_CLOSING_PERIOD_MS;
      mcLog2("Level crossing command CLOSED, servo increment " + String(levelCrossing.servoAngleIncrementPerMS * 1000) + " deg/s.", LOG_INFO);
      levelCrossing.lastStatusChangeTime_ms = millis();
      levelCrossing.boomBarrierActionInProgress = true;
      sendSensorEvent2MQTT(LEVEL_CROSSING_SENSOR_BOOMS_OPENED, false);
    }
  }
  else {
    mcLog2("Unkown levelCrossing command.", LOG_CRIT);
  }
}

void boomBarrierLoop() {
  const unsigned long BOOM_BARRIER_TICK_MS = 20;
  unsigned long now_ms = millis();

  if (now_ms < levelCrossing.lastBoomBarrierTick_ms + BOOM_BARRIER_TICK_MS) {
    return;
  }

  float servoAngleIncrement = levelCrossing.servoAngleIncrementPerMS * (now_ms - levelCrossing.lastBoomBarrierTick_ms);
  float newServoAnglePrimaryBooms;
  float newServoAngleSecondaryBooms;

  levelCrossing.lastBoomBarrierTick_ms = now_ms;

  if (!levelCrossing.boomBarrierActionInProgress) {
    return;
  }
  
  // Move primary booms?
  if ((levelCrossing.levelCrossingStatus == LevelCrossingStatus::OPEN || now_ms >= levelCrossing.lastStatusChangeTime_ms + LC_BOOM_BARRIER1_CLOSING_DELAY_MS) && levelCrossing.servoAnglePrimaryBooms != levelCrossing.servoTargetAnglePrimaryBooms) {
    if (levelCrossing.servoAnglePrimaryBooms < levelCrossing.servoTargetAnglePrimaryBooms) {
      newServoAnglePrimaryBooms = min(levelCrossing.servoAnglePrimaryBooms + servoAngleIncrement, levelCrossing.servoTargetAnglePrimaryBooms);
    }
    else {
      newServoAnglePrimaryBooms = max(levelCrossing.servoAnglePrimaryBooms - servoAngleIncrement, levelCrossing.servoTargetAnglePrimaryBooms);
    }
    mcLog2("Primary booms angle: " + String(newServoAnglePrimaryBooms), LOG_DEBUG);

    levelCrossing.servoAnglePrimaryBooms = newServoAnglePrimaryBooms;
  }

  // Move secondary booms?
  if ((levelCrossing.levelCrossingStatus == LevelCrossingStatus::OPEN || now_ms >= levelCrossing.lastStatusChangeTime_ms + LC_BOOM_BARRIER2_CLOSING_DELAY_MS) && levelCrossing.servoAngleSecondaryBooms != levelCrossing.servoTargetAngleSecondaryBooms) {
    // Yepp, move secondary booms!
    if (levelCrossing.servoAngleSecondaryBooms < levelCrossing.servoTargetAngleSecondaryBooms) {
      newServoAngleSecondaryBooms = min(levelCrossing.servoAngleSecondaryBooms + servoAngleIncrement, levelCrossing.servoTargetAngleSecondaryBooms);
    }
    else if (levelCrossing.servoAngleSecondaryBooms > levelCrossing.servoTargetAngleSecondaryBooms) {
      newServoAngleSecondaryBooms = max(levelCrossing.servoAngleSecondaryBooms - servoAngleIncrement, levelCrossing.servoTargetAngleSecondaryBooms);
    }
    mcLog2("Secondary booms angle: " + String(newServoAngleSecondaryBooms), LOG_DEBUG);

    levelCrossing.servoAngleSecondaryBooms = newServoAngleSecondaryBooms;
  }

  for (int bb = 0; bb < LC_NUM_BOOM_BARRIERS; bb++) {
    setServoAngle(LC_BOOM_BARRIER_SERVO_PIN[bb], (bb < 2) ? levelCrossing.servoAnglePrimaryBooms : levelCrossing.servoAngleSecondaryBooms);
  }

  // Final boom barrier position reached?
  if ((levelCrossing.servoAnglePrimaryBooms == levelCrossing.servoTargetAnglePrimaryBooms) && (levelCrossing.servoAngleSecondaryBooms == levelCrossing.servoTargetAngleSecondaryBooms)) {
    levelCrossing.boomBarrierActionInProgress = false;
    if (levelCrossing.levelCrossingStatus == LevelCrossingStatus::OPEN) {
      sendSensorEvent2MQTT(LEVEL_CROSSING_SENSOR_BOOMS_OPENED, true);
    }
    else {
      sendSensorEvent2MQTT(LEVEL_CROSSING_SENSOR_BOOMS_CLOSED, true);
    }
  }
}

void levelCrossingLightLoop() {
  // alternate all signal LEDs every LC_SIGNAL_FLASH_PERIOD_MS / 2 milliseconds
  unsigned long now_ms = millis();
  bool lightsActive = (levelCrossing.levelCrossingStatus == LevelCrossingStatus::CLOSED) || levelCrossing.boomBarrierActionInProgress;
  bool alternatePeriod = (now_ms % LC_SIGNAL_FLASH_PERIOD_MS) > (LC_SIGNAL_FLASH_PERIOD_MS / 2);

  for (int s = 0; s < LC_NUM_SIGNALS; s++) {
    if (LC_SIGNALS_FADING) {
      // fading lights
      int brightness = 0;
      if (lightsActive) {
        brightness = map(abs(LC_SIGNAL_FLASH_PERIOD_MS / 2 - ((now_ms + LC_SIGNAL_FLASH_PERIOD_MS * s / 2) % LC_SIGNAL_FLASH_PERIOD_MS)), 0, LC_SIGNAL_FLASH_PERIOD_MS / 2, -768, 1280);
      }
      fadeSignalLED(LC_SIGNAL_PIN[s], brightness);
    } else {
      // flashing lights
      setSignalLED(LC_SIGNAL_PIN[s], lightsActive && (((s % 2) == 0) ^ alternatePeriod));
    }
  }
}

// Handle level crossing sensor events
void handleLevelCrossingSensorEvent(int triggeredSensor) {
  if (!LEVEL_CROSSING_CONNECTED || !LC_AUTONOMOUS_MODE) return;

  mcLog2("Checking if sensor " + String(triggeredSensor) + " is a level crossing sensor...", LOG_DEBUG);

  // Iterate level crossing sensors
  for (int lcs = 0; lcs < LC_NUM_SENSORS; lcs++) {
    // Check if triggered sensors is level crossing sensor
    if (LC_SENSORS_INDEX[lcs] == triggeredSensor) {
      int track = LC_SENSORS_TRACK[lcs];
      int purposeConfig = LC_SENSORS_PURPOSE[lcs];
      int orientation = LC_SENSORS_ORIENTATION[lcs];

      mcLog2("> Sensor " + String(triggeredSensor) + " is a level crossing sensor, track " + String(track) + ", purpose " + String(purposeConfig) + ", orientation " + (String(orientation) ? "-" : "+"), LOG_DEBUG);

      int purposeFrom = purposeConfig;
      int purposeTo = purposeConfig;

      // sensor purpose "both" (inbound and outbound)?
      if (purposeConfig == 2) {
        purposeFrom = 0;
        purposeTo = 1;
      }

      for (int purpose = purposeFrom; purpose <= purposeTo; purpose++) {
        int count = ++(levelCrossing.sensorEventCounter[track][purpose][orientation]);

        // Inbound event?
        if (purpose == 0) {
          // Check if inbound counter is greater than inbound counter on the other side
          // -> train is approaching
          if (count > levelCrossing.sensorEventCounter[track][0][1 - orientation]) {
            // lock track
            levelCrossing.trackOccupied[track] = true;
            // close level crossing!
            levelCrossingCommand(1);
          }
        }
        // Outbound event?
        if (purpose == 1) {
          // Check if outbound counter equals the inbound counter on the other side
          // -> last car has passed the level crossing
          if (count == levelCrossing.sensorEventCounter[track][0][1 - orientation]) {
            // release track
            levelCrossing.trackOccupied[track] = false;
            // try to open level crossing
            levelCrossingCommand(0);
          }
        }
      }

      levelCrossing.trackOccupiedTimeout_ms[track] = millis() + LC_AUTONOMOUS_MODE_TRACK_TIMEOUT_MS;
      writeLevelCrossingStatusInfo();
    }
  }
}

// Returns if level crossing is occupied. Only relevant for autonomous mode
bool lcIsOccupied() {
  for (int t = 0; t < LC_NUM_TRACKS; t++) {
    if (levelCrossing.trackOccupied[t]) {
      return true;
    }
  }
  return false;
}

// Check if track timeout for Autonomous Mode has been reached. If reached, reset track counters and open level crossing.
void checkLevelCrossingTrackTimeouts() {
  for (int track = 0; track < LC_NUM_TRACKS; track++) {
    if (levelCrossing.trackOccupiedTimeout_ms[track] > 0) {
      if (millis() > levelCrossing.trackOccupiedTimeout_ms[track]) {
        // disable timeout for this track
        levelCrossing.trackOccupiedTimeout_ms[track] = 0;
        // reset counters for this track
        for (int purpose = 0; purpose < 2; purpose++) {
          for (int orientation = 0; orientation < 2; orientation++) {
            levelCrossing.sensorEventCounter[track][purpose][orientation] = 0;
          }
        }
        // release track
        levelCrossing.trackOccupied[track] = false;
        // try to open level crossing
        levelCrossingCommand(0);
        writeLevelCrossingStatusInfo();
      }
    }
  }
}

// write level crossing status to serial
void writeLevelCrossingStatusInfo() {
  if (LOGLEVEL_SERIAL >= LOG_DEBUG) {
    Serial.println("Trk|IB+|OB+|OB-|IB-");
    Serial.println("---+---+---+---+---");
    for (int track = 0; track < LC_NUM_TRACKS; track++) {
      Serial.print(" " + String(track + 1));
      Serial.print(" | " + String(levelCrossing.sensorEventCounter[track][0][0]));
      Serial.print(" | " + String(levelCrossing.sensorEventCounter[track][1][0]));
      Serial.print(" | " + String(levelCrossing.sensorEventCounter[track][1][1]));
      Serial.print(" | " + String(levelCrossing.sensorEventCounter[track][0][1]));
      Serial.println();
    }
  }
}

// main level crossing control loop
void levelCrossingLoop() {
  if (LEVEL_CROSSING_CONNECTED) {
    boomBarrierLoop();
    levelCrossingLightLoop();
    if (LC_AUTONOMOUS_MODE) {
      checkLevelCrossingTrackTimeouts();
    }
  }
}




// copy bascule bridge command to bridge object
void basculeBridgeCommand(int bridgeCommand) {
  if (bridgeCommand == 0) { // up
    mcLog2("Bascule bridge command UP.", LOG_INFO);
    resetBridgeLeafErrors();
    bridge.bridgeCommand = BridgeCommand::UP;
  }
  else if (bridgeCommand == 1) { // down
    mcLog2("Bascule bridge command DOWN.", LOG_INFO);
    resetBridgeLeafErrors();
    bridge.bridgeCommand = BridgeCommand::DOWN;
  }
  else {
    mcLog2("Unkown bascule bridge command.", LOG_CRIT);
  }
}

// set bridge motor power
void setBridgeMotorPower(int leafIndex, int motorPower) {
  mcLog2("[" + String(leafIndex) + "] Setting bridge motor power to " + String(motorPower), LOG_DEBUG);

  // limit motorPower input parameter to -100 .. 100
  if (motorPower > 100) {
    motorPower = 100;
  }
  else if (motorPower < -100) {
    motorPower = -100;
  }

  // PWM values for orange continuous servos: 0=full backward, 100=stop, 200=full forward
  setServoAngle(BASCULE_BRIDGE_SERVO_INDEX[leafIndex], motorPower + 100);
}

// set bridge lights
void setBridgeLights() {
  // set bridge lights
  bool blinkState = (millis() % 1000) >= 500;
  bool flashState = (millis() % 417) >= 208;

  switch (bridge.bridgeStatus) {
  case BridgeStatus::CLOSED:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, false);
    break;
  case BridgeStatus::OPENING:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, flashState);
    break;
  case BridgeStatus::OPENED:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, false);
    break;
  case BridgeStatus::CLOSING:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, flashState);
    break;
  case BridgeStatus::UNDEFINED:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, !blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, !blinkState);
    break;
  case BridgeStatus::ERRoR:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, flashState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, !blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, !flashState);
    break;
  }
}


// main bridge control loop
void basculeBridgeLoop() {
  if (!BASCULE_BRIDGE_CONNECTED)
    return;

  // Process bridge state machine
  // nextBridgeStatus == BridgeStatus::UNDEFINED means: no status change
  BridgeStatus nextBridgeStatus = BridgeStatus::UNDEFINED;

  if (bridge.bridgeCommand != BridgeCommand::NONE) {
    // Check for leaf errors
    if (bridge.bridgeStatus != BridgeStatus::ERRoR && checkForBridgeLeafErrors()) {
      // leaf error detected -> set bridge status to error and bridge command to none
      bridge.bridgeCommand = BridgeCommand::NONE;
      nextBridgeStatus = BridgeStatus::ERRoR;
    }
    else {
      // no leaf error -> process main bridge state machine
      switch (bridge.bridgeStatus) {
        case BridgeStatus::CLOSED:
          // in this state, usually nothing happenes unless the bridge command "up" is received
          if (bridge.bridgeCommand == BridgeCommand::UP) {
            nextBridgeStatus = BridgeStatus::OPENING;
          }
          break;
  
        case BridgeStatus::OPENING:
          if (bridge.bridgeCommand == BridgeCommand::DOWN) {
            nextBridgeStatus = BridgeStatus::CLOSING;
          }
          else if (checkAllBridgeLeafsOpened()) {
            nextBridgeStatus = BridgeStatus::OPENED;
          }
          break;
  
        case BridgeStatus::OPENED:
          // in this state, usually nothing happenes unless the bridge command "down" is received
          if (bridge.bridgeCommand == BridgeCommand::DOWN) {
            nextBridgeStatus = BridgeStatus::CLOSING;
          }
          break;
  
        case BridgeStatus::CLOSING:
          if (bridge.bridgeCommand == BridgeCommand::UP) {
            nextBridgeStatus = BridgeStatus::OPENING;
          }
          else if (checkAllBridgeLeafsClosed()) {
            nextBridgeStatus = BridgeStatus::CLOSED;
          }
          break;
  
        case BridgeStatus::UNDEFINED:
        case BridgeStatus::ERRoR:
          // if in error or undefined state, go to opening or closing state depending on the bridge command.
          if (bridge.bridgeCommand == BridgeCommand::UP) {
            nextBridgeStatus = BridgeStatus::OPENING;
          }
          else if (bridge.bridgeCommand == BridgeCommand::DOWN) {
            nextBridgeStatus = BridgeStatus::CLOSING;
          }
          break;
      }
    }
  }

  if (nextBridgeStatus != BridgeStatus::UNDEFINED) {
    bridge.bridgeStatus = nextBridgeStatus;

    switch (nextBridgeStatus) {
      case BridgeStatus::CLOSED:
        mcLog2("New bridge status: Bridge fully closed.", LOG_DEBUG);
        sendSensorEvent2MQTT(BASCULE_BRIDGE_SENSOR_FULLY_DOWN, true);
        break;

      case BridgeStatus::OPENING:
        mcLog2("New bridge status: Opening bridge...", LOG_DEBUG);
        sendSensorEvent2MQTT(BASCULE_BRIDGE_SENSOR_FULLY_DOWN, false);
        break;

      case BridgeStatus::OPENED:
        mcLog2("New bridge status: Bridge fully opened.", LOG_DEBUG);
        sendSensorEvent2MQTT(BASCULE_BRIDGE_SENSOR_FULLY_UP, true);
        break;

      case BridgeStatus::CLOSING:
        mcLog2("New bridge status: Closing bridge...", LOG_DEBUG);
        sendSensorEvent2MQTT(BASCULE_BRIDGE_SENSOR_FULLY_UP, false);
        break;

      case BridgeStatus::ERRoR:
        mcLog2("New bridge status: Bridge error.", LOG_DEBUG);
        sendSensorEvent2MQTT(BASCULE_BRIDGE_SENSOR_FULLY_DOWN, false);
        break;
    }
  }

  // Process leaf state machines
  processBridgeLeafs();

  // Update bridge lights
  setBridgeLights();
}

// Check if one or more bridge leafs are in error state
bool checkForBridgeLeafErrors() {
  for (int l = 0; l < NUM_BASCULE_BRIDGE_LEAFS; l++) {
    if (bridge.bridgeLeaf[l].leafStatus == BridgeLeafStatus::ERRoR) {
      return true;
    }
  }
  return false;
}

bool resetBridgeLeafErrors() {
  for (int l = 0; l < NUM_BASCULE_BRIDGE_LEAFS; l++) {
    if (bridge.bridgeLeaf[l].leafStatus == BridgeLeafStatus::ERRoR) {
      bridge.bridgeLeaf[l].leafStatus = BridgeLeafStatus::UNDEFINED;
    }
  }
}

// Check if all bridge leafs are in opened state
bool checkAllBridgeLeafsOpened() {
  for (int l = 0; l < NUM_BASCULE_BRIDGE_LEAFS; l++) {
    if (bridge.bridgeLeaf[l].leafStatus != BridgeLeafStatus::OPENED) {
      return false;
    }
  }
  return true;
}

// Check if all bridge leafs are in closed state
bool checkAllBridgeLeafsClosed() {
  for (int l = 0; l < NUM_BASCULE_BRIDGE_LEAFS; l++) {
    if (bridge.bridgeLeaf[l].leafStatus != BridgeLeafStatus::CLOSED) {
      return false;
    }
  }
  return true;
}

// Process bridge leaf state machines
bool processBridgeLeafs() {
  for (int l = 0; l < NUM_BASCULE_BRIDGE_LEAFS; l++) {
    processBridgeLeaf(l);
  }
}

// Process bridge leaf state machines
bool processBridgeLeaf(int leafIndex) {
  // get sensor states
  bool sensorUp = sensorState[BASCULE_BRIDGE_SENSOR_UP[leafIndex]];
  bool sensorDown = sensorState[BASCULE_BRIDGE_SENSOR_DOWN[leafIndex]];

  // Process bridge leaf state machine
  // nextLeafBridgeStatus == BridgeStatus::UNDEFINED means: no status change
  BridgeLeafStatus nextBridgeLeafStatus = BridgeLeafStatus::UNDEFINED;

  // The idea is to split the state machine into two branches depending on the bridge command  (up/down). This makes combined handling of several different states significantly easier.
  switch (bridge.bridgeCommand) {

    case BridgeCommand::UP:
      switch (bridge.bridgeLeaf[leafIndex].leafStatus) {
        case BridgeLeafStatus::OPENED:
          // do nothing
          break;
        case BridgeLeafStatus::CLOSING0:
          nextBridgeLeafStatus = BridgeLeafStatus::CLOSING1;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
        case BridgeLeafStatus::CLOSING1:
          if (sensorUp) {
            if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer >= BASCULE_BRIDGE_MAX_CLOSING_TIME_MS) {
              nextBridgeLeafStatus = BridgeLeafStatus::ERRoR;
              break;
            }
            break;
          }
          // fall-through
        case BridgeLeafStatus::CLOSING2:
        case BridgeLeafStatus::CLOSING3:
        case BridgeLeafStatus::CLOSED:
        case BridgeLeafStatus::UNDEFINED:
          if (!sensorUp) {
            nextBridgeLeafStatus = BridgeLeafStatus::OPENING0;
            bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          }
          else {
            nextBridgeLeafStatus = BridgeLeafStatus::CLOSING1;
            bridge.bridgeLeaf[leafIndex].leafTimer = millis();
            break;
          }
          // fall-through
        case BridgeLeafStatus::OPENING0:
          if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer < BASCULE_BRIDGE_LEAF_DELAY_OPEN_MS[leafIndex]) {
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::OPENING1;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          // fall-through
        case BridgeLeafStatus::OPENING1:
          if (sensorDown) {
            if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer >= BASCULE_BRIDGE_MAX_OPENING_TIME_MS) {
              nextBridgeLeafStatus = BridgeLeafStatus::ERRoR;
              break;
            }
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::OPENING2;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          // fall-through
        case BridgeLeafStatus::OPENING2:
          if (!sensorUp) {
            if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer >= BASCULE_BRIDGE_MAX_OPENING_TIME_MS) {
              nextBridgeLeafStatus = BridgeLeafStatus::ERRoR;
              break;
            }
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::OPENING3;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          // fall-through
        case BridgeLeafStatus::OPENING3:
          if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer >= BASCULE_BRIDGE_EXTRA_TIME_AFTER_OPENED_MS) {
            nextBridgeLeafStatus = BridgeLeafStatus::OPENED;
          }
          break;
      }
      break;

    case BridgeCommand::DOWN:
      switch (bridge.bridgeLeaf[leafIndex].leafStatus) {
        case BridgeLeafStatus::CLOSED:
          // do nothing
          break;
        case BridgeLeafStatus::OPENING0:
          if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer < BASCULE_BRIDGE_LEAF_DELAY_OPEN_MS[leafIndex]) {
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::OPENING1;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          // fall-through
        case BridgeLeafStatus::OPENING1:
          if (sensorDown) {
            if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer >= BASCULE_BRIDGE_MAX_OPENING_TIME_MS) {
              nextBridgeLeafStatus = BridgeLeafStatus::ERRoR;
              break;
            }
            break;
          }
          // fall-through
        case BridgeLeafStatus::OPENING2:
        case BridgeLeafStatus::OPENING3:
        case BridgeLeafStatus::OPENED:
          nextBridgeLeafStatus = BridgeLeafStatus::CLOSING0;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          break;
        case BridgeLeafStatus::UNDEFINED:
          // if bridge leaf is in undefined state, we can not determine if the bridge is really closed if the closed sensor is triggered
          // -> open bridge leaf until sensor is released for safety!
          if (!sensorDown) {
            nextBridgeLeafStatus = BridgeLeafStatus::CLOSING0;
            bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          }
          // if closed sensor not triggered, simply enter the standard closing sequence
          else {
            nextBridgeLeafStatus = BridgeLeafStatus::OPENING0;
            bridge.bridgeLeaf[leafIndex].leafTimer = millis();
            break;
          }
          // fall-through
        case BridgeLeafStatus::CLOSING0:
          if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer < BASCULE_BRIDGE_LEAF_DELAY_CLOSE_MS[leafIndex]) {
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::CLOSING1;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          // fall-through
        case BridgeLeafStatus::CLOSING1:
          if (sensorUp) {
            if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer >= BASCULE_BRIDGE_MAX_CLOSING_TIME_MS) {
              nextBridgeLeafStatus = BridgeLeafStatus::ERRoR;
              break;
            }
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::CLOSING2;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          // fall-through
        case BridgeLeafStatus::CLOSING2:
          if (!sensorDown) {
            if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer >= BASCULE_BRIDGE_MAX_CLOSING_TIME_MS) {
              nextBridgeLeafStatus = BridgeLeafStatus::ERRoR;
              break;
            }
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::CLOSING3;
          bridge.bridgeLeaf[leafIndex].leafTimer = millis();
          // fall-through
        case BridgeLeafStatus::CLOSING3:
          if (millis() - bridge.bridgeLeaf[leafIndex].leafTimer < BASCULE_BRIDGE_EXTRA_TIME_AFTER_CLOSED_MS) {
            break;
          }
          nextBridgeLeafStatus = BridgeLeafStatus::CLOSED;
          break;
      }
      break;

    case BridgeCommand::NONE:
      if (bridge.bridgeLeaf[leafIndex].leafStatus != BridgeLeafStatus::ERRoR) {
        nextBridgeLeafStatus = BridgeLeafStatus::ERRoR;
      }
      break;
  }

  // Process next bridge leaf status
  if (nextBridgeLeafStatus != BridgeLeafStatus::UNDEFINED) {
    // status change!
    bridge.bridgeLeaf[leafIndex].leafStatus = nextBridgeLeafStatus;
    bridge.bridgeLeaf[leafIndex].leafTimer = millis();

    switch (nextBridgeLeafStatus) {
      case BridgeLeafStatus::CLOSED:
        mcLog2("[" + String(leafIndex) + "] Bridge leaf fully closed.", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, 0);
        break;

      case BridgeLeafStatus::OPENING0:
        mcLog2("[" + String(leafIndex) + "] Bridge leaf standing by to be opened.", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, 0);
        break;
      case BridgeLeafStatus::OPENING1:
        mcLog2("[" + String(leafIndex) + "] Opening bridge leaf (initial stage)...", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, BASCULE_BRIDGE_POWER_UP);
      case BridgeLeafStatus::OPENING2:
        mcLog2("[" + String(leafIndex) + "] Opening bridge leaf (intermediate stage)...", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, BASCULE_BRIDGE_POWER_UP);
        break;
      case BridgeLeafStatus::OPENING3:
        mcLog2("[" + String(leafIndex) + "] Opening bridge leaf (final stage)...", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, BASCULE_BRIDGE_POWER_UP2);
        break;

      case BridgeLeafStatus::OPENED:
        mcLog2("[" + String(leafIndex) + "] Bridge leaf fully opened.", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, 0);
        break;

      case BridgeLeafStatus::CLOSING0:
        mcLog2("[" + String(leafIndex) + "] Bridge leaf standing by to be closed.", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, 0);
        break;
      case BridgeLeafStatus::CLOSING1:
        mcLog2("[" + String(leafIndex) + "] Closing bridge leaf (initial stage)...", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, -BASCULE_BRIDGE_POWER_DOWN);
        break;
      case BridgeLeafStatus::CLOSING2:
        mcLog2("[" + String(leafIndex) + "] Closing bridge leaf (intermediate stage)...", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, -BASCULE_BRIDGE_POWER_DOWN);
        break;
      case BridgeLeafStatus::CLOSING3:
        mcLog2("[" + String(leafIndex) + "] Closing bridge leaf (final stage)...", LOG_DEBUG);
        setBridgeMotorPower(leafIndex, -BASCULE_BRIDGE_POWER_DOWN2);
        break;

      case BridgeLeafStatus::UNDEFINED:
      case BridgeLeafStatus::ERRoR:
        mcLog2("[" + String(leafIndex) + "] Bridge leaf error.", LOG_CRIT);
        setBridgeMotorPower(leafIndex, 0);
        break;
    }
  }

  // Check if an emergency brake situation exists
  if (bridge.bridgeLeaf[leafIndex].leafStatus == BridgeLeafStatus::CLOSED) {
    if (!sensorDown) {
      bridge.bridgeLeaf[leafIndex].leafStatus = BridgeLeafStatus::ERRoR;
      mcLog2("ALERT: Bridge leaf " + String(leafIndex) + " is in status CLOSED, but the closing sensor was released!", LOG_ALERT);
      sendEmergencyBrake2MQTT("bridge unsafe - closing sensor unexpectetly released");
    }
  }
  else if (bridge.bridgeLeaf[leafIndex].leafStatus == BridgeLeafStatus::OPENED) {
    if (!sensorUp) {
      bridge.bridgeLeaf[leafIndex].leafStatus = BridgeLeafStatus::ERRoR;
      mcLog2("ALERT: Bridge leaf " + String(leafIndex) + " is in status OPENED, but the opening sensor was released!", LOG_ALERT);
    }
  }
  else if (sensorDown && sensorUp) {
    bridge.bridgeLeaf[leafIndex].leafStatus = BridgeLeafStatus::ERRoR;
    mcLog2("ALERT: Both sensors of bridge leaf " + String(leafIndex) + " triggered concurrently!", LOG_ALERT);
    sendEmergencyBrake2MQTT("bridge unsafe - leaf sensors triggered concurrently");
  }
}




void SpeedometerDebug() {
  mcLog2("Speedometer Debug ----------------------------------------------------", LOG_DEBUG);
  for (int i = 0; i <= speedometer.wheelcounter[speedometer.startSensor]; i++) {
    mcLog2("Magnet ["  + String(i) + "] Start: "  + String(speedometer.startTime[i]) + " End: " + String(speedometer.endTime[i]) + " Speed: "  + String(speedometer.trainSpeed[i]) + " Length: " + String(speedometer.trainLength[i]), LOG_DEBUG);
  }
  mcLog2("----------------------------------------------------------------------", LOG_DEBUG);
}

void updateDisplay() {
#if USE_U8G2
  float trainspeed  = speedometer.actualTrainSpeed;
  float trainlength = speedometer.actualTrainLength;
  String speedUnit;
  String lengthUnit;

  switch (speedometer.speedUnit) {
    case SpeedometerSpeedUnit::STUDS_PER_SECOND:
      trainspeed = trainspeed / 8;
      speedUnit = "studs/s";
      break;

    case SpeedometerSpeedUnit::MILLIMETERS_PER_SECOND:
      speedUnit = "mm/s";
      break;

    case SpeedometerSpeedUnit::KILOMETER_PER_HOUR:
      trainspeed = trainspeed * 3600 / 1000000;
      speedUnit = "km/h";
      break;

    case SpeedometerSpeedUnit::MILES_PER_HOUR:
      trainspeed = trainspeed * 3600 / 1609340;
      speedUnit = "studs";
      break;
  }

  switch (speedometer.lengthUnit) {
    case SpeedometerLengthUnit::STUDS:
      trainlength = trainlength / 8;
      lengthUnit = "studs";
      break;

    case SpeedometerLengthUnit::MILLIMETERS:
      lengthUnit = "mm";
      break;

    case SpeedometerLengthUnit::CENTIMETERS:
      trainlength = trainlength / 10;
      lengthUnit = "cm";
      break;

    case SpeedometerLengthUnit::METERS:
      trainlength = trainlength / 1000;
      lengthUnit = "m";
      break;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_12_me); // height: 8 pixels
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();

  u8g2.setCursor(5, 25);

  if (trainspeed <= 0) {
    u8g2.print("Speed: ? " + speedUnit);
  } else {
    u8g2.print("Speed: " + String((int)(trainspeed + 0.5)) + " " + speedUnit);
  }

  u8g2.setCursor(5, 50);
  if (speedometer.lengthUnit != SpeedometerLengthUnit::NO_INDICATION) {
    if (trainlength <= 0) {
      u8g2.print("Length: ? " + lengthUnit);
    } else {
      u8g2.print("Length: " + String((int)(trainlength + 0.5)) + " " + lengthUnit);
    }
  }

  u8g2.sendBuffer();
#endif
}


void handleSpeedometerSensorEvent(int triggeredSensor) {
  if (!SPEEDOMETER_CONNECTED) return;

  mcLog2("Checking if sensor " + String(triggeredSensor) + " is a speedometer sensor...", LOG_DEBUG);

  // Iterate speedometer sensors
  // Check if triggered sensors is a speedometer sensor
  bool isSpeedoMeterSensor = false;
  for (int sms = 0; sms < SM_NUM_SENSORS; sms++) {
    if (SM_SENSORS_INDEX[sms] == triggeredSensor) {
       //Serial.println("sensor " + String(triggeredSensor) + " is a speedometer sensor...");
       isSpeedoMeterSensor = true;
       break;
    }
  }

  if (!isSpeedoMeterSensor) return;

  if (speedometer.occupied) {

  //-----------------------------------------------------
  // Speedometer is occupied, handle startSensor-Events
  //-----------------------------------------------------
    if (speedometer.startSensor == triggeredSensor) {
      speedometer.lastMeasurementEvent = millis();
      int wcStart = ++speedometer.wheelcounter[speedometer.startSensor];

      speedometer.startTime[wcStart]   = speedometer.lastMeasurementEvent;
      speedometer.endTime[wcStart]     = 0;
      speedometer.trainSpeed[wcStart]  = 0;
      speedometer.trainLength[wcStart] = 0;
    }

  //---------------------------------------------------
  // Speedometer is occupied, handle endSensor-Events
  //---------------------------------------------------
    if (speedometer.endSensor == triggeredSensor) {
      speedometer.lastMeasurementEvent = millis();
      int wcEnd = ++speedometer.wheelcounter[speedometer.endSensor];

      // if there are missing startSensor events, we stay at the current line of the startSensor
      if (speedometer.wheelcounter[speedometer.endSensor] > speedometer.wheelcounter[speedometer.startSensor]) {
        // Serial.println(String(speedometer.wheelcounter[speedometer.endSensor]) + " adapted to " + StringSpeedometer.wheelcounter[speedometer.startSensor]));
        speedometer.wheelcounter[speedometer.endSensor] = speedometer.wheelcounter[speedometer.startSensor];
      }

      speedometer.endTime[wcEnd] = speedometer.lastMeasurementEvent;

      float timeDiffSpeed  = (speedometer.endTime[wcEnd] - speedometer.startTime[wcEnd]) / 1000;
      speedometer.trainSpeed[wcEnd] = SM_DISTANCE / timeDiffSpeed;

      if (wcEnd == 0) {
        speedometer.trainLength[wcEnd]   = 0;
      } else {
        float avgSpeed       = (speedometer.trainSpeed[wcEnd] + speedometer.trainSpeed[wcEnd - 1]) / 2;
        float timeDiffLength = (speedometer.endTime[wcEnd]    - speedometer.endTime[wcEnd - 1]);
        speedometer.trainLength[wcEnd] = avgSpeed * timeDiffLength / 1000;
      }

      if (wcEnd == speedometer.wheelcounter[speedometer.startSensor]) {
        speedometer.actualTrainSpeed = 0;
        speedometer.actualTrainLength = 0;

        for (int i = 0; i <= wcEnd; i++) {
          speedometer.actualTrainSpeed  += speedometer.trainSpeed[i];
          speedometer.actualTrainLength += speedometer.trainLength[i];
        }
        speedometer.actualTrainSpeed = speedometer.actualTrainSpeed / (wcEnd + 1);

        mcLog2("actualTrainSpeed:  " + String(speedometer.actualTrainSpeed), LOG_DEBUG);
        mcLog2("actualTrainLength: " + String(speedometer.actualTrainLength), LOG_DEBUG);
      }
      
      SpeedometerDebug();
    }

    // no length measurement or wheelcounter similar or timeout
    if (millis() >= speedometer.lastMeasurementEvent + SM_TIMEOUT){
      mcLog2("Speedometer reset (2)!", LOG_INFO);
      speedometer.occupied = false;
      speedometer.wheelcounter[speedometer.startSensor] = -1;
      speedometer.wheelcounter[speedometer.endSensor]   = -1;

      speedometer.measurementDone = millis();
    }
  }

  //-------------------------------------------------
  // Speedometer is free, handle startSensor-Events
  //-------------------------------------------------
  if (!speedometer.occupied && millis() - speedometer.measurementDone >= SM_TIME_BETWEEN_MEASUREMENTS) {
    speedometer.lastMeasurementEvent = millis();

    speedometer.startSensor = triggeredSensor;
    speedometer.endSensor   = (SM_NUM_SENSORS - 1) - triggeredSensor;
    speedometer.occupied = true;
    speedometer.wheelcounter[speedometer.startSensor] = 0;
    speedometer.wheelcounter[speedometer.endSensor]   = -1;

    speedometer.startTime[speedometer.wheelcounter[speedometer.startSensor]]   = speedometer.lastMeasurementEvent;
    speedometer.endTime[speedometer.wheelcounter[speedometer.startSensor]]     = 0;
    speedometer.trainSpeed[speedometer.wheelcounter[speedometer.startSensor]]  = 0;
    speedometer.trainLength[speedometer.wheelcounter[speedometer.startSensor]] = 0;
  }
}


void speedometerLoop() {
  if (!SPEEDOMETER_CONNECTED) return;

  if (speedometer.occupied) {
    // no length measurement or wheelcounter smilar or timeout
    if (millis() >= speedometer.lastMeasurementEvent + SM_TIMEOUT) {
      mcLog2("Speedometer reset (1)!", LOG_INFO);
      speedometer.occupied = false;
      speedometer.wheelcounter[speedometer.startSensor] = -1;
      speedometer.wheelcounter[speedometer.endSensor]   = -1;
      speedometer.measurementDone = millis();
    }

    if (millis() - speedometer.lastMeasurementEvent >= 500) {
      updateDisplay();
    }
  }

//----------------------------------------------------------------------------------------------------------------------------

  unsigned long actMillis = millis();

  if (!speedometer.occupied && actMillis - speedometer.measurementDone < SM_TIME_BETWEEN_MEASUREMENTS && actMillis - speedometer.lastMeasurementEvent > 1000) {
      int remaningDuration = (SM_TIME_BETWEEN_MEASUREMENTS - (millis() - speedometer.measurementDone)) / 1000;
      if ((remaningDuration < 5 || remaningDuration % 5 == 0) && remaningDuration > 0){
        mcLog2("Minimum time between measurements: " + String((int)(SM_TIME_BETWEEN_MEASUREMENTS - (millis() - speedometer.measurementDone)) / 1000) + " seconds remaining", LOG_DEBUG);
        speedometer.lastMeasurementEvent = actMillis;
      }
    }

//----------------------------------------------------------------------------------------------------------------------------

#if USE_U8G2
  // display the very cool MattzoBricks screensaver
  if (!speedometer.occupied && actMillis - speedometer.measurementDone >= SM_TIME_TO_SHOW_RESULTS && actMillis - speedometer.animationDelay >= 100) {
    static byte rotor = 0;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_unifont_t_symbols);

    switch (rotor % 4){
      case 0:
        u8g2.drawStr(10, 35, "MattzoBricks |");
        break;
      case 1:
        u8g2.drawStr(10, 35, "MattzoBricks /");
        break;
      case 2:
        u8g2.drawStr(10, 35, "MattzoBricks -");
        break;
      case 3:
        u8g2.drawStr(10, 35, "MattzoBricks \\");
        break;
    }
    u8g2.sendBuffer();

    rotor = (++rotor % 4);
    speedometer.animationDelay = actMillis;
  }
#endif
}


void loop() {
  loopMattzoController();
  checkEnableServoSleepMode();
  monitorSensors();
  levelCrossingLoop();
  basculeBridgeLoop();
  speedometerLoop();
}
