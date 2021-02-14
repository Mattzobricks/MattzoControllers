// MattzoSwitchController Firmware
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define MATTZO_CONTROLLER_TYPE "MattzoLayoutController"
#include <ESP8266WiFi.h>                          // WiFi library for ESP-8266
#include <Servo.h>                                // Servo library
#include "MattzoLayoutController_Configuration_BasculeBridge.h" // this file should be placed in the same folder
#include "MattzoController_Library.h"             // this file needs to be placed in the Arduino library folder

#if USE_PCA9685
#include <Wire.h>                                 // Built-in library for I2C
#include <Adafruit_PWMServoDriver.h>              // Adafruit PWM Servo Driver Library for PCA9685 port expander. Tested with version 2.4.0.
#endif

// SERVO VARIABLES AND CONSTANTS

// Create servo objects to control servos
Servo servo[NUM_SWITCHPORTS];

// Create PWM Servo Driver object (for PCA9685)
#if USE_PCA9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();  // only board 0x40 is supported in this version of the firmware
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
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


// BASCULE BRIDGE VARIABLES AND CONSTANTS
enum struct BridgeStatus
{
  STOPPED = 0x1,
  CLOSED = 0x2,
  OPENING = 0x3,
  OPEN = 0x4,
  CLOSING = 0x5,
  CLOSING2 = 0x6,
  ERROR = 0x7,             // error condition
  NO_TIMED_EVENT = 0x10,   // for timed events: no timed event pending
  COMMAND_PENDING = 0x11   // a bridge command was received, which has not been used to set a new bridge state
};

enum struct BridgeCommand
{
  UP = 0,
  DOWN = 1,
};

struct Bridge {
  BridgeStatus bridgeStatus = BridgeStatus:COMMAND_PENDING;
  BridgeStatus nextBridgeStatus = BridgeStatus:NO_TIMED_EVENT;
  unsigned long nextEventTime_ms;
  BridgeCommand bridgeCommand = BridgeCommand::DOWN;
  int motorPower = 0;
} bridge;


void setup() {
  // initialize PWM Servo Driver object (for PCA9685)
#if USE_PCA9685
  setupPCA9685();

  // Switch PCA9685 off
  if (PCA9685_OE_PIN_INSTALLED) {
    pinMode(PCA9685_OE_PIN, OUTPUT);
    setServoSleepMode(true);
  }
#endif

  // initialize servo pins and turn servos to start position
  for (int i = 0; i < NUM_SWITCHPORTS; i++) {
    if (SWITCHPORT_PIN_TYPE[i] == 0) {
      // servo connected directly to the controller
      servo[i].attach(SWITCHPORT_PIN[i]);
      servo[i].write(SERVO_START);
      delay(SWITCH_DELAY);
    }
    else if (SWITCHPORT_PIN_TYPE[i] == 0x40) {
      // servo connected to PCA9685
      // no action required
    }
  }

  // initialize signal pins
  for (int i = 0; i < NUM_SIGNALPORTS; i++) {
    if (SIGNALPORT_PIN_TYPE[i] == 0) {
      // signal connected directly to the controller
      pinMode(SIGNALPORT_PIN[i], OUTPUT);
    }
    else if (SIGNALPORT_PIN_TYPE[i] == 0x40) {
      // signal connected to PCA9685
      // no action required
    }
  }

  // initialize sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PIN[i], INPUT_PULLUP);
    sensorState[i] = false;
    sensorTriggerState[i] = (SENSOR_PIN[i] == D8) ? HIGH : LOW;
  }

  // load config from EEPROM, initialize Wifi, MQTT etc.
  setupMattzoController();
}

#if USE_PCA9685
void setupPCA9685() {
  // Initialize PWM Servo Driver object (for PCA9685)
  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(SERVO_FREQ);
  delay(10);
}
#endif

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  msg[length] = '\0';

  mcLog("Received MQTT message [" + String(topic) + "]: " + String(msg));

  XMLDocument xmlDocument;
  if(xmlDocument.Parse(msg)!= XML_SUCCESS){
    mcLog("Error parsing.");
    return;
  }
  mcLog("Parsing XML successful.");

  XMLElement *element;
  element = xmlDocument.FirstChildElement("sw");
  if (element != NULL) {
    mcLog("<sw> node found.");

    // query addr1 attribute. This is the MattzoController id.
    // If this does not equal the mattzoControllerId of this controller, the message is disregarded.
    int rr_addr1 = 0;
    if (element->QueryIntAttribute("addr1", &rr_addr1) != XML_SUCCESS) {
      mcLog("addr1 attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("addr1: " + String(rr_addr1));
    if (rr_addr1 != mattzoControllerId) {
      mcLog("Message disgarded, as it is not for me (" + String(mattzoControllerId) + ")");
      return;
    }

    // query port1 attribute. This is port id of the port to which the switch is connected.
    // If the controller does not have such a port, the message is disregarded.
    int rr_port1 = 0;
    if (element->QueryIntAttribute("port1", &rr_port1) != XML_SUCCESS) {
      mcLog("port1 attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("port1: " + String(rr_port1));
    if (BASCULE_BRIDGE_CONNECTED && (rr_port1 == BASCULE_BRIDGE_RR_PORT)) {
      mcLog("This is a bascule bridge command.");
    }
    else if ((rr_port1 < 1 || rr_port1 > NUM_SWITCHPORTS) && (rr_port1 < 1001)) {
      mcLog("Message disgarded, this controller does not have such a port.");
      return;
    }

    // query cmd attribute. This is the desired switch setting and can either be "turnout" or "straight".
    const char* rr_cmd = "-unknown-";
    if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
      mcLog("cmd attribute not found or wrong type.");
      return;
    }
    mcLog("cmd: " + String(rr_cmd));

    // query param1 attribute. This is the "straight" position of the switch servo motor.
    // defaults to SERVO_MIN
    int rr_param1 = SERVO_MIN;
    if (element->QueryIntAttribute("param1", &rr_param1) != XML_SUCCESS) {
      mcLog("param1 attribute not found or wrong type. Using default value.");
    }
    if (rr_param1 < SERVO_MIN_ALLOWED || rr_param1 > SERVO_MAX_ALLOWED) {
      // Reset angle back to standard if angle is too small
      // User has obviously forgotten to configure servo angle in Rocrail properly
      // To protect the servo, the default value is used
      mcLog("param1 attribute out of bounds. Using default value.");
      rr_param1 = SERVO_MIN;
    }
    mcLog("param1: " + String(rr_param1));

    // query value1 attribute. This is the "turnout" position of the switch servo motor.
    // defaults to SERVO_MAX
    int rr_value1 = SERVO_MAX;
    if (element->QueryIntAttribute("value1", &rr_value1) != XML_SUCCESS) {
      mcLog("value1 attribute not found or wrong type. Using default value.");
    }
    if (rr_value1 < SERVO_MIN_ALLOWED || rr_value1 > SERVO_MAX_ALLOWED) {
      // Reset angle back to standard if angle is too small
      // User has obviously forgotten to configure servo angle in Rocrail properly
      // To protect the servo, the default value is used
      mcLog("value1 attribute out of bounds. Using default value.");
      rr_value1 = SERVO_MAX;
    }
    mcLog("value1: " + String(rr_value1));

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
      mcLog("Switch command unknown - message disregarded.");
      return;
    }

    // Check if port is used to control a bascule bridge
    if (BASCULE_BRIDGE_CONNECTED && (rr_port1 == BASCULE_BRIDGE_RR_PORT)) {
      basculeBridgeCommand(switchCommand);
    }

    // Check if port is a logical port
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

          mcLog("Turning servos on logical port " + String(rr_port1) + "...");
          setServoAngle(servoPort1, servoAngle1);
          setServoAngle(servoPort2, servoAngle2);
        }
      }

      if (!logicalSwitchPortFound) {
        mcLog("Error: logical port " + String(rr_port1) + " unknown.");
      }
    }
    else {
      setServoAngle(rr_port1 - 1, (switchCommand == 1) ? rr_param1 : rr_value1);
    }
    return;
    // end of switch command handling
  }

  element = xmlDocument.FirstChildElement("co");
  if (element != NULL) {
    mcLog("<co> node found.");

    // query addr attribute. This is the MattzoController id.
    // If this does not equal the ControllerNo of this controller, the message is disregarded.
    int rr_addr = 0;
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog("addr attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("addr: " + String(rr_addr));
    if (rr_addr != mattzoControllerId) {
      mcLog("Message disgarded, as it is not for me (" + String(mattzoControllerId) + ")");
      return;
    }

    // query port attribute. This is port id of the port to which the switch is connected.
    // If the controller does not have such a port, the message is disregarded.
    int rr_port = 0;
    if (element->QueryIntAttribute("port", &rr_port) != XML_SUCCESS) {
      mcLog("port attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("port: " + String(rr_port));
    if (rr_port < 1 || rr_port > NUM_SIGNALPORTS) {
      mcLog("Message disgarded, as this controller does not have such a port.");
      return;
    }

    // query cmd attribute. This is the desired switch setting and can either be "turnout" or "straight".
    const char* rr_cmd = "-unknown-";
    if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
      mcLog("cmd attribute not found or wrong type.");
      return;
    }
    mcLog("cmd: " + String(rr_cmd));

    // set signal LED for the port on/off
    if (strcmp(rr_cmd, "on") == 0) {
      setSignalLED(rr_port - 1, true);
    }
    else if (strcmp(rr_cmd, "off") == 0) {
      setSignalLED(rr_port - 1, false);
    }
    else {
      mcLog("Signal port command unknown - message disregarded.");
    }
    return;
    // end of signal handling
  }

  mcLog("No <sw> or <co> node found. Message disregarded.");
}

void sendSensorEvent2MQTT(int sensorPort, int sensorState) {
  String sensorRocId = MATTZO_CONTROLLER_TYPE + String(mattzoControllerId) + "-" + String(sensorPort + 1);  // e.g. "MattzoController12345-3"
  String stateString = sensorState ? "true" : "false";

  // compile mqtt message. Parameters:
  //   id: Combination of sensor name and port (e.g. MattzoController12345-3). The reported port (the "logic" port) is 1 count higher than the internal port number in the sensor, e.g. port 2 in the sensor equals 3 in Rocrail)
  //   bus: controller number
  //   address: port number (internal port number plus 1)
  // both id or bus/address can be used in Rocrail. If id is used, it superseeds the combination of bus and address
  String mqttMessage = "<fb id=\"" + sensorRocId + "\" bus=\"" + String(mattzoControllerId) + "\" addr=\"" + String(sensorPort + 1) + "\" state=\"" + stateString + "\"/>";
  mcLog("Sending MQTT message: " + mqttMessage);
  char mqttMessage_char[255];   // message is usually 61 chars, so 255 chars should be enough
  mqttMessage.toCharArray(mqttMessage_char, mqttMessage.length() + 1);
  mqttClient.publish("rocrail/service/client", mqttMessage_char);
}

void sendEmergencyBrake2MQTT(String emergencyBrakeReason) {
  String mqttMessage = "<sys cmd=\"ebreak\" reason=\"" + emergencyBrakeReason + "\"/>";
  mcLog("Sending emergency brake message via MQTT: " + mqttMessage);
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
    int sensorValue = digitalRead(SENSOR_PIN[i]);

    if (sensorValue == sensorTriggerState[i]) {
      // Contact -> report contact immediately
      if (!sensorState[i]) {
        mcLog("Sensor " + String(i) + " triggered.");
        sendSensorEvent2MQTT(i, true);
        sensorState[i] = true;
      }
      lastSensorContact_ms[i] = millis();
    }
    else {
      // No contact for SENSOR_RELEASE_TICKS milliseconds -> report sensor has lost contact
      if (sensorState[i] && (millis() > lastSensorContact_ms[i] + SENSOR_RELEASE_TICKS)) {
        mcLog("Sensor " + String(i) + " released.");
        sendSensorEvent2MQTT(i, false);
        sensorState[i] = false;
      }
    }
  }

  setLEDBySensorStates();
}

// sets the servo arm to a desired angle
void setServoAngle(int servoIndex, int servoAngle) {
  mcLog("Turning servo index " + String(servoIndex) + " to angle " + String(servoAngle));
  if (servoIndex >= 0 && servoIndex < NUM_SWITCHPORTS) {
    if (SWITCHPORT_PIN_TYPE[servoIndex] == 0) {
      servo[servoIndex].write(servoAngle);
    }
#if USE_PCA9685
    else if (SWITCHPORT_PIN_TYPE[servoIndex] == 0x40) {
      setServoSleepMode(false);
      pca9685.setPWM(SWITCHPORT_PIN[servoIndex], 0, mapAngle2PulseLength(servoAngle));
    }
#endif
    delay(SWITCH_DELAY);
  }
  else {
    // this should not happen
    mcLog("WARNING: servo index " + String(servoIndex) + " out of range!");
  }
}

// converts a desired servo angle (0� - 180�) into a pwm pulse length (required for PCA9685)
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

// switches a signal on or off
void setSignalLED(int signalIndex, bool ledState) {
  mcLog("Setting signal LED " + String(signalIndex) + " to " + String(ledState));
  if (SIGNALPORT_PIN_TYPE[signalIndex] == 0) {
    digitalWrite(SIGNALPORT_PIN[signalIndex], ledState ? LOW : HIGH);
  }
#if USE_PCA9685
  else if (SIGNALPORT_PIN_TYPE[signalIndex] == 0x40) {
    if (ledState) {
      // full bright
      pca9685.setPWM(SIGNALPORT_PIN[signalIndex], 4096, 0);
      // half bright (strongly dimmed)
      // pca9685.setPWM(SIGNALPORT_PIN[signalIndex], 0, 2048);
      // 3/4 bright (slightly dimmed)
      // pca9685.setPWM(SIGNALPORT_PIN[signalIndex], 0, 3072);
    }
    else {
      // off
      pca9685.setPWM(SIGNALPORT_PIN[signalIndex], 0, 4096);
    }
  }
#endif
}


// copy bascule bridge command to bridge object
void basculeBridgeCommand(int bridgeCommand) {
  if (bridgeCommand == 0) { // up
    mcLog2("Bascule bridge command UP.", LOG_DEBUG);
    bridge.bridgeCommand = BridgeCommand::UP;
    bridge.bridgeStatus = case BridgeStatus::COMMAND_PENDING:
  }
  else if (bridgeCommand == 1) { // down
    mcLog2("Bascule bridge command DOWN.", LOG_DEBUG);
    bridge.bridgeCommand = BridgeCommand::DOWN;
    bridge.bridgeStatus = case BridgeStatus::COMMAND_PENDING:
  }
  else {
    mcLog2("Unkown bascule bridge command.", LOG_CRIT);
  }
}

// set bridge motor power
void setBridgeMotorPower(int motorPower) {
  bridge.motorPower = motorPower;
  mcLog2("Setting bridge motor power to " + String(motorPower), LOG_DEBUG);
}

// set bridge lights
void setBridgeLights() {
  // set bridge lights
  bool blinkState = (millis() / 1000) >= 500;
  bool flashState = (millis() / 417) >= 208;

  switch (newBridgeStatus) {
  case BridgeStatus::STOPPED:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, blinkState);
    break;
  case BridgeStatus::CLOSED:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, false);
    break;
  case BridgeStatus::OPENING:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, blinkState);
    break;
  case BridgeStatus::OPEN:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, false);
    break;
  case BridgeStatus::CLOSING:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, true);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, blinkState);
    break;
  case BridgeStatus::CLOSING2:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, false);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, flashState);
    break;
  case BridgeStatus::ERROR:
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_STOP, flashState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_PREP, !flashState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_RIVER_GO, blinkState);
    setSignalLED(BASCULE_BRIDGE_SIGNAL_BLINK_LIGHT, !blinkState);
    break;
  }
}


// main bridge control loop
void basculeBridgeLoop() {
  // 1. Determine if bridge state change is required
  BridgeStatus newBridgeStatus;

  // Timed state change?
  if (bridge.bridgeStatus != bridge.nextBridgeStatus && bridge.nextBridgeStatus != NO_TIMED_EVENT && millis() >= bridge.nextEventTime_ms) {
    newBridgeStatus = bridge.nextBridgeStatus;
  } else {
    // no timed state change required.
    // Check if a state change is required due to sensor event or bridge command.
    newBridgeStatus = bridge.bridgeStatus;

    if (bridge.bridgeCommand = BridgeCommand::UP) {
      // Bridge command: UP!

      switch (bridge.bridgeStatus) {
      case BridgeStatus::CLOSED:
      case BridgeStatus::CLOSING:
      case BridgeStatus::CLOSING2:
      case BridgeStatus::COMMAND_PENDING:
        // Check if "bridge up" sensor is triggered.
        if (sensorState[BASCULE_BRIDGE_SENSOR_UP]) {
          mcLog2("Bridge was already open.", LOG_DEBUG);
          newBridgeStatus = BridgeStatus::OPEN;
        }
        // Open bridge!
        else {
          mcLog2("Opening bridge...", LOG_DEBUG);
          newBridgeStatus = BridgeStatus::OPENING;
          // make sure bridge motors stops after BASCULE_BRIDGE_MAX_OPENING_TIME_MS of "Sensor Up" was not triggered until then.
          bridge.nextBridgeStatus = BridgeStatus::STOPPED;
          bridge.nextEventTime_ms = millis() + BASCULE_BRIDGE_MAX_OPENING_TIME_MS;
        }
        break;

      case BridgeStatus::OPENING:
        if (sensorState[BASCULE_BRIDGE_SENSOR_UP]) {
          mcLog2("Bridge is now open.", LOG_DEBUG);
          newBridgeStatus = BridgeStatus::OPEN;
        }
        break;
      default:
        mcLog2("Unknown bridge status.", LOG_CRIT);
      }
    }
    else {
      // Bridge command: DOWN!

      switch (bridge.bridgeStatus) {
      case BridgeStatus::CLOSING:
        if (sensorState[BASCULE_BRIDGE_SENSOR_DOWN]) {
          if (BASCULE_BRIDGE_EXTRA_TIME_AFTER_CLOSED_MS > 0) {
            // continue closing for BASCULE_BRIDGE_EXTRA_TIME_AFTER_CLOSED_MS milliseconds
            newBridgeStatus = BridgeStatus::CLOSING2;
            // add timed state change
            bridge.nextBridgeStatus = BridgeStatus::CLOSED;
            bridge.nextEventTime_ms = millis() + BASCULE_BRIDGE_EXTRA_TIME_AFTER_CLOSED_MS;
          }
          else {
            newBridgeStatus = BridgeStatus::CLOSED;
          }
        }
        break;

      case BridgeStatus::OPENING:
      case BridgeStatus::OPEN:
      case BridgeStatus::COMMAND_PENDING:
        // Check if "bridge down" sensor is still triggered.
        if (sensorState[BASCULE_BRIDGE_SENSOR_DOWN]) {
          mcLog2("Bridge was already down.", LOG_DEBUG);
          newBridgeStatus = BridgeStatus::CLOSED;
        }
        // Close bridge!
        else {
          mcLog2("Closing bridge...", LOG_DEBUG);
          newBridgeStatus = BridgeStatus::CLOSING;
          // make sure bridge motors stops after BASCULE_BRIDGE_MAX_CLOSING_TIME_MS of "Sensor Down" was not triggered until then.
          bridge.nextBridgeStatus = BridgeStatus::STOPPED;
          bridge.nextEventTime_ms = millis() + BASCULE_BRIDGE_MAX_CLOSING_TIME_MS;
        }
        break;
      default:
        mcLog2("Unknown bridge status.", LOG_CRIT);
      }
    }
  }

  // 2. Execute bridge state change
  // Bridge state change pending?
  if (newBridgeStatus != bridge.bridgeStatus) {
    bridge.bridgeStatus = newBridgeStatus;

    switch (newBridgeStatus) {
    case BridgeStatus::STOPPED:
    case BridgeStatus::CLOSED:
    case BridgeStatus::OPEN:
    case BridgeStatus::ERROR:
      setBridgeMotorPower(0);
      break;
    case BridgeStatus::OPENING:
      setBridgeMotorPower(BASCULE_BRIDGE_POWER_UP);
      break;
    case BridgeStatus::CLOSING:
      setBridgeMotorPower(BASCULE_BRIDGE_POWER_DOWN);
      break;
    case BridgeStatus::CLOSING2:
      setBridgeMotorPower(BASCULE_BRIDGE_POWER_DOWN2);
      break;
    default:
      mcLog2("Unknown bridge status.", LOG_CRIT);
    }
  }

  // 3. Check if an emergency brake situation exists
  if (bridge.bridgeStatus == BridgeStatus::CLOSED) {
    if (!sensorState[BASCULE_BRIDGE_SENSOR_DOWN]) {
      bridge.bridgeStatus = BridgeStatus::ERROR;
      mcLog2("Alert: Bridge open!", LOG_ALERT);
      sendEmergencyBrake2MQTT("bridge open");
    }
    else if (sensorState[BASCULE_BRIDGE_SENSOR_DOWN] && sensorState[BASCULE_BRIDGE_SENSOR_UP]) {
      bridge.bridgeStatus = BridgeStatus::ERROR;
      mcLog2("Alert: Bridge sensors triggered concurrently!", LOG_ALERT);
      sendEmergencyBrake2MQTT("bridge sensors triggered concurrently");
    }
  }

  // 4. Update bridge lights
  setBridgeLights();
}


void loop() {
  loopMattzoController();
  checkEnableServoSleepMode();
  monitorSensors();
  basculeBridgeLoop();
}
