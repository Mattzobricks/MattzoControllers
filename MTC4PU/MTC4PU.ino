// Author: Dr. Matthias Runte
// Copyright 2020, 2021 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// ****************************************
// TARGET-PLATTFORM for this sketch: ESP-32
// ****************************************

// Constants for Powered Up Lights connected to Powered Up Units.
const uint8_t PU_LIGHT = 253;

// MattzoControllerType
#define MATTZO_CONTROLLER_TYPE "MTC4PU"

// Built-in libraries of the Arduino IDE
#include <WiFi.h>          // WiFi library for ESP-32

// Legoino library by Cornelius Munz
// Install via the built-in Library Manager of the Arduino IDE
// Tested with:
//   Version V1.1.0 of the "Legoino" library
//   Version V1.0.2 of the required dependent library "NimBLE-Arduino"
// To connect more than 3 Powered Up units to the ESP-32, the constant CONFIG_BT_NIMBLE_MAX_CONNECTIONS
//   needs to be changed in nimconfig.h in the NimBLE-Arduino/src directory of your libraries folder
//   An ESP-32 can handle 9 BT connections concurrently.
// Make this line: #define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 3
// Look like this: #define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 9
#include "Lpf2Hub.h"

// devices that may be connected to a Powered Up hub port
enum struct MattzoPUDevice {
  NONE = 0x0,
  PU_MOTOR = 0x1,
  PU_LIGHT = 0x2
};

struct MattzoLocoConfiguration {
  String locoName;
  int locoAddress;
  int accelerationInterval;
  int accelerateStep;
  int brakeStep;
};

struct MattzoPUHubConfiguration {
  String hubName;
  String macAddress;
  MattzoPUDevice devicePortA;
  int configMotorA;
  MattzoPUDevice devicePortB;
  int configMotorB;
  int locoAddress;
};

// Forward declaration
class MattzoLoco;
class MattzoPUHub;

// MattzoBricks library files
#include "MTC4PU_Configuration.h"     // this file should be placed in the same folder
#include "MattzoController_Library.h" // this file needs to be placed in the Arduino library folder



// Class containing a locomotive object as defined in Rocrail
class MattzoLoco {
public:
  // Members
  String _locoName;                             // name of the loco as specified in Rocrail
  int _locoAddress;                             // address of the loco in Rocrail
  int _currentTrainSpeed = 0;                   // current speed of this train
  int _targetTrainSpeed = 0;                    // Target speed of this train
  int _maxTrainSpeed = 0;                       // Maximum speed of this train as configured in Rocrail
  unsigned long _lastAccelerate = millis();     // time of the last speed adjustment

  // Motor acceleration parameters
  int _accelerationInterval = 100;       // pause between individual speed adjustments in milliseconds
  int _accelerateStep = 1;               // acceleration increment for a single acceleration step
  int _brakeStep = 2;                    // brake decrement for a single braking step

  // Methods
  void initMattzoLoco(MattzoLocoConfiguration c) {
    _locoName = c.locoName;
    _locoAddress = c.locoAddress;
    _accelerationInterval = c.accelerationInterval;
    _accelerateStep = c.accelerateStep;
    _brakeStep = c.brakeStep;
  };

  String getNiceName() {
    return _locoName + " (" + _locoAddress + ")";
  }
} myLocos[NUM_LOCOS];  // Objects for MattzoLocos


// Class containining the Powered Hub hub object from the Legoino library
class MattzoPUHub {
public:
  // Static constants
  static const int DELAY = 10;  // a small delay in ms after calling functions on the Legoino library (else the Legoino library may crash)
  static const int MAX_PU_POWER = 88;  // maximum useful powered up power
  static const int PU_SCAN_DURATION = 1000;  // time the controller tries to connect to a Hub (in ms)

  // Members
  Lpf2Hub legoinoHub;  // The hub object from Legoino
  String _hubName;
  String _macAddress;
  MattzoPUDevice _devicePortA;
  int _configMotorA;  // 1 = forward, -1 = reverse
  MattzoPUDevice _devicePortB;
  int _configMotorB;  // 1 = forward, -1 = reverse
  int _locoAddress;   // address of the Rocrail loco in which this PU hub is built-in
  bool lastKnownConnectionStatus = false;

  // Methods
  void initMattzoPUHub(MattzoPUHubConfiguration c) {
    _hubName = c.hubName;
    _macAddress = c.macAddress;
    _devicePortA = c.devicePortA;
    _configMotorA = c.configMotorA;
    _devicePortB = c.devicePortB;
    _configMotorB = c.configMotorB;
    _locoAddress = c.locoAddress;
  };

  String getNiceName() {
    return _hubName + " (" + _macAddress + ")";
  }

  void initHub() {
    char macAddress_char[18];
    if (_macAddress.length() != 17) {
      mcLog2("Can not initialize hub " + getNiceName() + ": error in mac address", LOG_CRIT);
    }
    _macAddress.toCharArray(macAddress_char, 18);
    mcLog2("Initializing hub " + getNiceName() + "...", LOG_INFO);
    legoinoHub.init(macAddress_char, PU_SCAN_DURATION);
    delay(DELAY);
  }

  bool isConnecting() {
    return legoinoHub.isConnecting();
  }

  bool isConnected() {
    return legoinoHub.isConnected();
  }

  void requestBatteryReport(HubPropertyChangeCallback hubPropertyChangeCallback) {
    if (isConnected()) {
      mcLog2("Requesting battery level for hub " + getNiceName() + "...", LOG_DEBUG);
      legoinoHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
      delay(DELAY);
    }
  }

  bool checkLocoAddress(int locoAddress) {
    return (locoAddress == 0 || locoAddress == _locoAddress);
  }

  // Set integrated powered up hub light
  void setLedColor(Color ledColor, int locoAddress) {
    if (!checkLocoAddress(locoAddress)) return;

    if (!isConnected()) return;

    legoinoHub.setLedColor(ledColor);
    delay(DELAY);
  }

  // Set motor power
  void setMotorSpeed(int power, int locoAddress) {
    if (!checkLocoAddress(locoAddress)) return;

    if (!isConnected()) return;

    if (_devicePortA == MattzoPUDevice::PU_MOTOR) {
      legoinoHub.setBasicMotorSpeed((byte)PoweredUpHubPort::A, power * _configMotorA);
      delay(DELAY);
    }
    if (_devicePortB == MattzoPUDevice::PU_MOTOR) {
      legoinoHub.setBasicMotorSpeed((byte)PoweredUpHubPort::B, power * _configMotorB);
      delay(DELAY);
    }
  }

  // Set light intensity for connected Powered Up hubs
  void setLights(int lightPower, int locoAddress) {
    if (!checkLocoAddress(locoAddress)) return;

    if (!isConnected()) return;

    if (_devicePortA == MattzoPUDevice::PU_LIGHT) {
      legoinoHub.setBasicMotorSpeed((byte)PoweredUpHubPort::A, lightPower);
      delay(DELAY);
    }
    if (_devicePortB == MattzoPUDevice::PU_LIGHT) {
      legoinoHub.setBasicMotorSpeed((byte)PoweredUpHubPort::B, lightPower);
      delay(DELAY);
    }
  }
} myHubs[NUM_HUBS];  // Objects for Powered Up Hubs

/* Functions */
bool functionCommand[NUM_FUNCTIONS];  // Desired state of a function
bool functionState[NUM_FUNCTIONS];    // Actual state of a function

enum struct LightEventType
{
  STOP = 0x0,
  FORWARD = 0x1,
  REVERSE = 0x2
};

/* Legoino Library */
String discoveryHubAddress;   // Will be send to MQTT as soon connection to MQTT is established.
int initializedHub = -1;   // The presently initialized hub. Only one hub can be initialized at once. :-(

// Send battery level
const bool REPORT_BATTERYLEVEL = false;          // set to true or false to allow or omit battery level reports
const int SEND_BATTERYLEVEL_INTERVAL = 60000;   // interval for sending battery level in milliseconds (60000 = 60 seconds)
unsigned long lastBatteryLevelMsg = millis();   // time of the last sent battery level
int nextBatteryLevelReportingHub = 0;           // index of the last hubs for which a battery report was requested

// Global emergency brake flag.
boolean ebreak = false;



void setup() {
  // initialize function pins
  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    // only real (non-virtual) pins shall be initialized
    if (FUNCTION_PIN[i] < PU_LIGHT) {
      pinMode(FUNCTION_PIN[i], OUTPUT);
    }
    functionCommand[i] = false;
    functionState[i] = false;
  }

  // load loco configuration
  MattzoLocoConfiguration *locoConf = getMattzoLocoConfiguration();
  for (int i = 0; i < NUM_LOCOS; i++) {
    myLocos[i].initMattzoLoco(*(locoConf + i));
  }

  // load Powered Up hub configuration
  MattzoPUHubConfiguration *hubConf = getMattzoPUHubConfiguration();
  for (int i = 0; i < NUM_HUBS; i++) {
    myHubs[i].initMattzoPUHub(*(hubConf + i));
  }

  // load config from EEPROM, initialize Wifi, MQTT etc.
  setupMattzoController();

  // discover new Powered Up Hubs (just to display MAC address)
  discoverPoweredUpHub();
}



int getMattzoLocoIndexByLocoAddress(int locoAddress) {
  mcLog2("getMattzoLocoIndexByLocoAddress is checking if this controller handles loco " + String(locoAddress) + "...", LOG_DEBUG);
  for (int l = 0; l < NUM_LOCOS; l++) {
    if (myLocos[l]._locoAddress == locoAddress) {
      return l;
    }
  }
  return -1;
}


void discoverPoweredUpHub() {
  // Just after booting, the ESP-32 checks if there is a Powered Up hub that is ready to connect
  // Purpose is just to read out its MAC address at publish it via MQTT.
  // The MAC address is then known and can be used in the configuration of the MattzoController (recompiling the source code is required)

  Lpf2Hub discoveryHub;  // Hub used for discovering new PU hubs

  mcLog2("Discovering Powered Up Hubs...", LOG_INFO);
  discoveryHub.init();
  unsigned long timeStart = millis();
  while (millis() - timeStart < 5000) {  // wait 5 seconds for a hub to show up
    if (discoveryHub.isConnecting()) {
      discoveryHubAddress = discoveryHub.getHubAddress().toString().c_str();
      mcLog2("Powered Up Hub found. MAC Address: " + discoveryHubAddress, LOG_INFO);
      return;
    }
  }
  mcLog2("No Powered Up Hub found. Discovering new Powered Up hubs terminated.", LOG_INFO);
}

void connectPoweredUpHubs() {
  // check for hubs ready for connection (is connecting, but not connected) -> connect
  for (int i = 0; i < NUM_HUBS; i++) {
    if (!myHubs[i].isConnected()) {
      // has this hub been initialized?
      if (i != initializedHub) {
        myHubs[i].initHub();
        initializedHub = i;
      }

      // hub connecting -> connect!
      if (myHubs[i].isConnecting()) {
        // Connect to hub
        mcLog("Connecting to hub " + String(i) + "...");
        myHubs[i].legoinoHub.connectHub();

        if (myHubs[i].isConnected()) {
          // Send "connected" message
          mcLog("Connected to hub " + String(i) + ".");
          sendMQTTMessage("roc2bricks/connectionStatus", myHubs[i].getNiceName() + " connected");
          myHubs[i].lastKnownConnectionStatus = true;
        }
        else {
          mcLog2("Connection attempt to hub " + String(i) + " refused.", LOG_ERR);
          myHubs[i].initHub();
        }
      }
      else {
        if (myHubs[i].lastKnownConnectionStatus) {
          // Send "connection lost" message
          mcLog("Hub " + String(i) + " disconnected.");
          sendMQTTMessage("roc2bricks/connectionStatus", myHubs[i].getNiceName() + " disconnected");
          myHubs[i].lastKnownConnectionStatus = false;
          myHubs[i].initHub();
        }
      }

      // if unconnected hub found, do not process any other hubs at this time.
      break;
    }
  }
}

void requestPoweredUpBatteryLevel() {
  if (REPORT_BATTERYLEVEL && mqttClient.connected()) {
    if (millis() - lastBatteryLevelMsg >= SEND_BATTERYLEVEL_INTERVAL / NUM_HUBS) {
      lastBatteryLevelMsg = millis();

      if (myHubs[nextBatteryLevelReportingHub].isConnected()) {
        mcLog2("Requesting battery level for hub " + String(nextBatteryLevelReportingHub), LOG_DEBUG);
        myHubs[nextBatteryLevelReportingHub].requestBatteryReport(hubPropertyChangeCallback);
        delay(10);
      }

      nextBatteryLevelReportingHub = (++nextBatteryLevelReportingHub) % NUM_HUBS;
    }
  }
}

void hubPropertyChangeCallback(void* hub, HubPropertyReference hubProperty, uint8_t* pData)
{
  Lpf2Hub* myHub = (Lpf2Hub*)hub;
  String hubAddress = myHub->getHubAddress().toString().c_str();
  mcLog("PU Message received from " + hubAddress + ", hub property " + String((byte)hubProperty, HEX));

  if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE)
  {
    mcLog("BatteryLevel: " + String(myHub->parseBatteryLevel(pData), DEC));
    sendMQTTMessage("roc2bricks/battery", hubAddress + " " + myHub->parseBatteryLevel(pData));
    return;
  }

  if (hubProperty == HubPropertyReference::BUTTON)
  {
    mcLog("Button: " + String((byte)myHub->parseHubButton(pData), HEX));
    sendMQTTMessage("roc2bricks/button", hubAddress + " button pressed.");
    return;
  }
}


void sendMQTTMessage(String topic, String message) {
  const int MAX_MQTT_TOPIC_SIZE = 255;
  const int MAX_MQTT_MESSAGE_SIZE = 255;
  char topic_char[MAX_MQTT_TOPIC_SIZE + 1];
  char message_char[MAX_MQTT_MESSAGE_SIZE + 1];

  if (topic.length() + 1 > MAX_MQTT_TOPIC_SIZE) {
    mcLog2("ERROR: MQTT topic string too long - message not sent.", LOG_CRIT);
    return;
  }

  message = String(mattzoControllerName) + " " + message;
  if (message.length() + 1 > MAX_MQTT_MESSAGE_SIZE) {
    mcLog2("ERROR: MQTT message string too long - message not sent.", LOG_CRIT);
    return;
  }

  topic.toCharArray(topic_char, topic.length() + 1);
  message.toCharArray(message_char, message.length() + 1);

  mcLog2("sending mqtt: " + topic + " " + message, LOG_CRIT);
  mqttClient.publish(topic_char, message_char);
}

// if a Powered Hub was discovered on start-up, transmit its MAC address once after initial MQTT connection.
void sendDiscoveredHub2mqtt() {
  if (discoveryHubAddress.length() > 0) {
    if (getConnectionStatus() == MCConnectionStatus::CONNECTED) {
      sendMQTTMessage("roc2bricks/discovery", "Powered Up Hub discovered: " + discoveryHubAddress);
      discoveryHubAddress = "";
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  msg[length] = '\0';

  mcLog2("Received MQTT message [" + String(topic) + "]: " + String(msg), LOG_DEBUG);

  XMLDocument xmlDocument;
  if (xmlDocument.Parse(msg) != XML_SUCCESS) {
    mcLog2("Error parsing.", LOG_ERR);
    return;
  }

  mcLog2("Parsing XML successful.", LOG_DEBUG);

  const char* rr_id = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";
  int rr_addr = 0;

  // check for lc message
  XMLElement* element = xmlDocument.FirstChildElement("lc");
  if (element != NULL) {
    mcLog2("<lc> node found. Processing loco message...", LOG_DEBUG);

    // -> process lc (loco) message

    // query id attribute. This is the loco name.
    // The id is a mandatory field. If not found, the message is discarded.
    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      mcLog2("id attribute not found or wrong type.", LOG_ERR);
      return;
    }
    mcLog2("loco id: " + String(rr_id), LOG_DEBUG);

    // query addr attribute. This is the address of the loco as specified in Rocrail.
    // Must match the locoAddress of the train object.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog2("addr attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("addr: " + String(rr_addr), LOG_DEBUG);

    int locoIndex = getMattzoLocoIndexByLocoAddress(rr_addr);
    if (locoIndex < 0) {
      mcLog2("Message disregarded, as this controller does not handle train " + String(rr_addr), LOG_DEBUG);
      return;
    }
    MattzoLoco& loco = myLocos[locoIndex];
    mcLog2("Consuming message for train " + loco.getNiceName(), LOG_DEBUG);

    // query dir attribute. This is the direction information for the loco (forward, reverse)
    const char* rr_dir = "xxxxxx";  // expected values are "true" or "false"
    int dir;
    if (element->QueryStringAttribute("dir", &rr_dir) != XML_SUCCESS) {
      mcLog2("dir attribute not found or wrong type.", LOG_ERR);
      return;
    }
    mcLog2("dir (raw): " + String(rr_dir), LOG_DEBUG);
    if (strcmp(rr_dir, "true") == 0) {
      mcLog2("direction: forward", LOG_DEBUG);
      dir = 1;
    }
    else if (strcmp(rr_dir, "false") == 0) {
      mcLog2("direction: backward", LOG_DEBUG);
      dir = -1;
    }
    else {
      mcLog2("unknown dir value - disregarding message.", LOG_ERR);
      return;
    }

    // query V attribute. This is the speed information for the loco and ranges from 0 to V_max (see below).
    int rr_v = 0;
    if (element->QueryIntAttribute("V", &rr_v) != XML_SUCCESS) {
      mcLog2("V attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("V: " + String(rr_v), LOG_DEBUG);

    // query V_max attribute. This is maximum speed of the loco. It must be set in the loco settings in Rocrail as percentage value.
    // The V_max attribute is required to map to loco speed from rocrail to a power setting in the MattzoController.
    int rr_vmax = 0;
    if (element->QueryIntAttribute("V_max", &rr_vmax) != XML_SUCCESS) {
      mcLog2("V_max attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("V_max: " + String(rr_vmax), LOG_DEBUG);

    // set target train speed
    loco._targetTrainSpeed = rr_v * dir;
    loco._maxTrainSpeed = rr_vmax;
    mcLog2("Loco " + loco.getNiceName() + ": Target speed set to " + String(loco._targetTrainSpeed) + " (current: " + String(loco._currentTrainSpeed) + ", max: " + String(loco._maxTrainSpeed) + ")", LOG_INFO);

    return;
  }

  // Check for fn message
  element = xmlDocument.FirstChildElement("fn");
  if (element != NULL) {
    mcLog2("<fn> node found. Processing fn message...", LOG_DEBUG);

    // -> process fn (function) message

    // query id attribute. This is the loco name.
    // The id is a mandatory field. If not found, the message is discarded.
    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      mcLog2("id attribute not found or wrong type.", LOG_ERR);
      return;
    }
    mcLog2("function id: " + String(rr_id), LOG_DEBUG);

    // query addr attribute. This is the address of the loco as specified in Rocrail.
    // Must match the locoAddress of the train object.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog2("addr attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("addr: " + String(rr_addr), LOG_DEBUG);

    int locoIndex = getMattzoLocoIndexByLocoAddress(rr_addr);
    if (locoIndex < 0) {
      mcLog2("Message disregarded, as this controller does not handle train " + String(rr_addr), LOG_DEBUG);
      return;
    }
    MattzoLoco& loco = myLocos[locoIndex];
    mcLog2("Consuming message for train " + loco.getNiceName(), LOG_DEBUG);

    // query fnchanged attribute. This is information which function shall be set.
    int rr_functionNo;
    if (element->QueryIntAttribute("fnchanged", &rr_functionNo) != XML_SUCCESS) {
      mcLog2("fnchanged attribute not found or wrong type. Message disregarded.", LOG_ERR);
      return;
    }
    mcLog2("fnchanged: " + String(rr_functionNo), LOG_DEBUG);

    if (rr_functionNo < 1 || rr_functionNo > NUM_FUNCTIONS) {
      mcLog2("fnchanged out of range. Message disregarded.", LOG_ERR);
      return;
    }
    int functionPinId = rr_functionNo - 1;
    mcLog2("Function PIN Id: " + String(functionPinId), LOG_DEBUG);

    // Check if the function is associated with the loco
    if (FUNCTION_PIN_LOCO_ADDRESS[functionPinId] != loco._locoAddress) {
      mcLog2("Function PIN is associated with loco " + String(FUNCTION_PIN_LOCO_ADDRESS[functionPinId]) + ", not with " + String(loco._locoAddress) + ". Disregarding message.", LOG_DEBUG);
      return;
    }

    // query fnchangedstate attribute. This is value if the function shall be set on or off
    const char* rr_state = "xxxxxx";  // expected values are "true" or "false"
    if (element->QueryStringAttribute("fnchangedstate", &rr_state) != XML_SUCCESS) {
      mcLog2("fnchangedstate attribute not found or wrong type.", LOG_ERR);
      return;
    }
    mcLog2("fnchangedstate (raw): " + String(rr_state), LOG_DEBUG);
    if (strcmp(rr_state, "true") == 0) {
      mcLog2("Loco " + loco.getNiceName() + ": setting function id " + String(functionPinId) + " to true", LOG_INFO);
      functionCommand[functionPinId] = true;
    }
    else if (strcmp(rr_state, "false") == 0) {
      mcLog2("Loco " + loco.getNiceName() + ": setting function id " + String(functionPinId) + " to false", LOG_INFO);
      functionCommand[functionPinId] = false;
    }
    else {
      mcLog2("unknown fnchangedstate value - disregarding message.", LOG_ERR);
      return;
    }

    return;
  }

  // Check for sys message
  element = xmlDocument.FirstChildElement("sys");
  if (element != NULL) {
    mcLog2("<sys> node found. Processing sys message...", LOG_DEBUG);

    const char* rr_cmd = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";

    // query cmd attribute. This is the system message type.
    if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
      mcLog2("cmd attribute not found or wrong type.", LOG_DEBUG);
      return;
    }

    String rr_cmd_s = String(rr_cmd);
    mcLog2("rocrail system command: " + String(rr_cmd_s), LOG_DEBUG);

    // Upon receiving "stop", "ebreak" or "shutdown" system command from Rocrail, the global emergency break flag is set. Train will stop immediately.
    // Upon receiving "go" command, the emergency break flag is be released (i.e. pressing the light bulb in Rocview).

    if (rr_cmd_s == "ebreak" || rr_cmd_s == "stop" || rr_cmd_s == "shutdown") {
      mcLog2("received ebreak, stop or shutdown command. Stopping train.", LOG_INFO);
      ebreak = true;
    }
    else if (rr_cmd_s == "go") {
      mcLog2("received go command. Releasing emergency break.", LOG_INFO);
      ebreak = false;
    }
    else {
      mcLog2("received other system command, disregarded.", LOG_DEBUG);
    }
    return;
  }

  mcLog2("Unknown message, disregarded.", LOG_DEBUG);
}

// set powered up motor speed
void setTrainSpeed(int newTrainSpeed, int locoIndex) {
  MattzoLoco& loco = myLocos[locoIndex];

  // set motor power
  int power = map(newTrainSpeed, 0, loco._maxTrainSpeed, 0, MattzoPUHub::MAX_PU_POWER);
  mcLog2("Setting motor speed: " + String(newTrainSpeed) + " (power: " + String(power) + ")", LOG_DEBUG);
  for (int i = 0; i < NUM_HUBS; i++) {
    myHubs[i].setMotorSpeed(power, loco._locoAddress);
  }

  // Execute light events
  if (newTrainSpeed == 0 && loco._currentTrainSpeed != 0) {
    lightEvent(LightEventType::STOP, locoIndex);
  }
  else if (newTrainSpeed > 0 && loco._currentTrainSpeed <= 0) {
    lightEvent(LightEventType::FORWARD, locoIndex);
  }
  else if (newTrainSpeed < 0 && loco._currentTrainSpeed >= 0) {
    lightEvent(LightEventType::REVERSE, locoIndex);
  }

  loco._currentTrainSpeed = newTrainSpeed;

  // Set integrated powered up hub light according to situation
  Color ledColor;
  if (loco._currentTrainSpeed != loco._targetTrainSpeed) {
    // accelerating / braking
    ledColor = YELLOW;
  }
  else if (loco._currentTrainSpeed == 0) {
    // stopped
    ledColor = RED;
  }
  else {
    // travelling at target speed
    ledColor = GREEN;
  }

  for (int i = 0; i < NUM_HUBS; i++) {
    myHubs[i].setLedColor(ledColor, loco._locoAddress);
  }
}


// gently adapt train speed (increase/decrease slowly)
void accelerateTrainSpeed() {
  boolean accelerateFlag;
  int step;
  int nextSpeed;

  for (int locoIndex = 0; locoIndex < NUM_LOCOS; locoIndex++) {
    MattzoLoco& loco = myLocos[locoIndex];

    if (ebreak) {
      // emergency break pulled and train moving -> stop train immediately
      if (loco._currentTrainSpeed != 0) {
        setTrainSpeed(0, locoIndex);
      }
    }
    else if (loco._currentTrainSpeed != loco._targetTrainSpeed) {
      if (loco._targetTrainSpeed == 0) {
        // stop -> execute immediately
        setTrainSpeed(0, locoIndex);
      }
      else if (millis() - loco._lastAccelerate >= loco._accelerationInterval) {
        loco._lastAccelerate = millis();
  
        // determine if trains accelerates or brakes
        accelerateFlag = abs(loco._currentTrainSpeed) < abs(loco._targetTrainSpeed) && (loco._currentTrainSpeed * loco._targetTrainSpeed > 0);
        step = accelerateFlag ? loco._accelerateStep : loco._brakeStep;
  
        // accelerate / brake gently
        if (loco._currentTrainSpeed < loco._targetTrainSpeed) {
          nextSpeed = min(loco._currentTrainSpeed + step, loco._targetTrainSpeed);
        }
        else {
          nextSpeed = max(loco._currentTrainSpeed - step, loco._targetTrainSpeed);
        }
        setTrainSpeed(nextSpeed, locoIndex);
      }
    }
  }
}

// execute light event
// IF DESIRED, YOU MAY UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
void lightEvent(LightEventType le, int locoIndex) {
  if (!AUTO_LIGHTS)
    return;

  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    if (locoIndex == 0 || locoIndex == FUNCTION_PIN_LOCO_ADDRESS[i]) {
      switch (le) {
      case LightEventType::STOP:
        mcLog2("Light event stop", LOG_DEBUG);
        // switch all functions off
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        functionCommand[i] = false;
        break;
      case LightEventType::FORWARD:
        mcLog2("Light event forward", LOG_DEBUG);
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        functionCommand[i] = (i % 2) == 0;
        break;
      case LightEventType::REVERSE:
        mcLog2("Light event reverse", LOG_DEBUG);
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        functionCommand[i] = (i % 2) == 1;
        break;
      }
    }
  }
}

// switch lights on or off
void setLights() {
  int puLightPower;

  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    bool onOff = functionCommand[i];  // desired on/off state of the LED
    bool lightStateChange = false;

    if (ebreak) {
      // override LED on ebreak (alternate lights on/off every 500ms)
      long phase = (millis() / 500) % 2;
      onOff = (phase + i) % 2 == 0;
    }

    if (onOff != functionState[i]) {
      mcLog2("Flipping function " + String(i + 1) + " to " + String(onOff), LOG_DEBUG);
      functionState[i] = onOff;
      switch (FUNCTION_PIN[i]) {
      case PU_LIGHT:
        puLightPower = onOff ? 100 : 0;
        for (int h = 0; h < NUM_HUBS; h++) {
          myHubs[h].setLights(puLightPower, FUNCTION_PIN_LOCO_ADDRESS[i]);
        }
        break;
      default:
        digitalWrite(FUNCTION_PIN[i], onOff ? HIGH : LOW);
      } // of switch
    } // of if
  } // of for
}



void loop() {
  loopMattzoController();
  sendDiscoveredHub2mqtt();
  connectPoweredUpHubs();
  requestPoweredUpBatteryLevel();
  accelerateTrainSpeed();
  setLights();

  debugInfo();
}


















// From here: debugging code! May be removed later
unsigned long timeLastHubReport = 0;

void debugInfo() {
  // return;  // if not commented out, connection information is put out every 10 seconds

  if (millis() - timeLastHubReport < 10000) return;
  timeLastHubReport = millis();

  mcLog("*** DEBUG-INFO ***");
  mcLog("WiFi + MQTT connection Status: " + String(getConnectionStatus() == MCConnectionStatus::CONNECTED));
  for (int i = 0; i < NUM_LOCOS; i++) {
    mcLog("Loco " + String(i) + ": " + myLocos[i].getNiceName() + ". Target speed: " + String(myLocos[i]._targetTrainSpeed) + ", current speed: " + String(myLocos[i]._currentTrainSpeed) + ", max speed: " + String(myLocos[i]._maxTrainSpeed));
  }
  mcLog("next hub to initialize (if any): " + String(initializedHub));
  for (int i = 0; i < NUM_HUBS; i++) {
    mcLog("Hub " + String(i) + ": macAddress=" + myHubs[i]._macAddress + ": isConnecting()=" + String(myHubs[i].isConnecting()) + " isConnected()=" + String(myHubs[i].isConnected()) + " status=" + String(myHubs[i].lastKnownConnectionStatus));
  }
  mcLog("******************");
}
