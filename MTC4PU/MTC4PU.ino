// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
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
      mcLog("Can not initialize hub " + getNiceName() + ": error in mac address");
    }
    _macAddress.toCharArray(macAddress_char, 18);
    mcLog("Initializing hub " + getNiceName() + "...");
    legoinoHub.init(macAddress_char, PU_SCAN_DURATION);
    delay(10);
  }

  bool isConnecting() {
    return legoinoHub.isConnecting();
  }

  bool isConnected() {
    return legoinoHub.isConnected();
  }

  void requestBatteryReport(HubPropertyChangeCallback hubPropertyChangeCallback) {
    if (isConnected()) {
      mcLog("Requesting battery level for hub " + getNiceName() + "...");
      legoinoHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
      delay(10);
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
const int SEND_BATTERYLEVEL_INTERVAL = 60000; // interval for sending battery level in milliseconds (60000 = 60 seconds)
unsigned long lastBatteryLevelMsg = millis();    // time of the last sent battery level
int nextBatteryLevelReportingHub = 0;            // index of the last hubs for which a battery report was requested

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
  // mcLog("getMattzoLocoIndexByLocoAddress is checking if this controller handles loco " + String(locoAddress) + "...");
  for (int l = 0; l < NUM_LOCOS; l++) {
    if (myLocos[l]._locoAddress == locoAddress) {
      // mcLog("getMattzoLocoIndexByLocoAddress found loco " + myLocos[l].getNiceName() + " (index " + String(l) + ").");
      return l;
    }
  }
  // mcLog("getMattzoLocoIndexByLocoAddress did not find the loco.");
  return -1;
}



void discoverPoweredUpHub() {
  // Just after booting, the ESP-32 checks if there is a Powered Up hub that is ready to connect
  // Purpose is just to read out its MAC address at publish it via MQTT.
  // The MAC address is then known and can be used in the configuration of the MattzoController (recompiling the source code is required)

  Lpf2Hub discoveryHub;  // Hub used for discovering new PU hubs

  mcLog("Discovering Powered Up Hubs...");
  discoveryHub.init();
  unsigned long timeStart = millis();
  while (millis() - timeStart < 5000) {  // wait 5 seconds for a hub to show up
    if (discoveryHub.isConnecting()) {
      discoveryHubAddress = discoveryHub.getHubAddress().toString().c_str();
      mcLog("Powered Up Hub found. MAC Address: " + discoveryHubAddress);
      return;
    }
  }
  mcLog("No Powered Up Hub found. Discovering new Powered Up hubs terminated.");
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
          mcLog("Connection attempt to hub " + String(i) + " refused.");
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
  if (millis() - lastBatteryLevelMsg >= SEND_BATTERYLEVEL_INTERVAL / NUM_HUBS) {
    lastBatteryLevelMsg = millis();

    if (myHubs[nextBatteryLevelReportingHub].isConnected()) {
      // mcLog("Requesting battery level for hub " + String(nextBatteryLevelReportingHub));
      myHubs[nextBatteryLevelReportingHub].requestBatteryReport(hubPropertyChangeCallback);
      delay(10);
    }

    nextBatteryLevelReportingHub = (++nextBatteryLevelReportingHub) % NUM_HUBS;
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
    mcLog("ERROR: MQTT topic string too long - message not sent.");
    return;
  }

  message = String(mattzoControllerName) + " " + message;
  if (message.length() + 1 > MAX_MQTT_MESSAGE_SIZE) {
    mcLog("ERROR: MQTT message string too long - message not sent.");
    return;
  }

  topic.toCharArray(topic_char, topic.length() + 1);
  message.toCharArray(message_char, message.length() + 1);

  mcLog("sending mqtt: " + topic + " " + message);
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

  mcLog("Received MQTT message [" + String(topic) + "]: " + String(msg));

  XMLDocument xmlDocument;
  if (xmlDocument.Parse(msg) != XML_SUCCESS) {
    mcLog("Error parsing.");
    return;
  }

  mcLog("Parsing XML successful.");

  const char* rr_id = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";
  int rr_addr = 0;

  // check for lc message
  XMLElement* element = xmlDocument.FirstChildElement("lc");
  if (element != NULL) {
    mcLog("<lc> node found. Processing loco message...");

    // -> process lc (loco) message

    // query id attribute. This is the loco id.
    // The id is a mandatory field. If not found, the message is discarded.

    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      mcLog("id attribute not found or wrong type.");
      return;
    }
    mcLog("loco id: " + String(rr_id));

    // query addr attribute. This is the MattzoController id.
    // If this does not equal the LOCO_ADDRESS of this controller, the message is disregarded.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog("addr attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("addr: " + String(rr_addr));

    int locoIndex = getMattzoLocoIndexByLocoAddress(rr_addr);
    if (locoIndex < 0) {
      mcLog("Message disregarded, as this controller does not handle train " + String(rr_addr));
      return;
    }
    MattzoLoco& loco = myLocos[locoIndex];
    mcLog("Consuming message for train " + loco.getNiceName());

    // query dir attribute. This is direction information for the loco (forward, backward)
    const char* rr_dir = "xxxxxx";  // expected values are "true" or "false"
    int dir;
    if (element->QueryStringAttribute("dir", &rr_dir) != XML_SUCCESS) {
      mcLog("dir attribute not found or wrong type.");
      return;
    }
    mcLog("dir (raw): " + String(rr_dir));
    if (strcmp(rr_dir, "true") == 0) {
      mcLog("direction: forward");
      dir = 1;
    }
    else if (strcmp(rr_dir, "false") == 0) {
      mcLog("direction: backward");
      dir = -1;
    }
    else {
      mcLog("unknown dir value - disregarding message.");
      return;
    }

    // query V attribute. This is the speed information for the loco and ranges from 0 to V_max (see below).
    int rr_v = 0;
    if (element->QueryIntAttribute("V", &rr_v) != XML_SUCCESS) {
      mcLog("V attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("V: " + String(rr_v));

    // query V_max attribute. This is maximum speed of the loco. It must be set in the loco settings in Rocrail as percentage value.
    // The V_max attribute is required to map to loco speed from rocrail to a power setting in the MattzoController.
    int rr_vmax = 0;
    if (element->QueryIntAttribute("V_max", &rr_vmax) != XML_SUCCESS) {
      mcLog("V_max attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("V_max: " + String(rr_vmax));

    // set target train speed
    loco._targetTrainSpeed = rr_v * dir;
    loco._maxTrainSpeed = rr_vmax;
    mcLog("Message parsing complete, target speed set to " + String(loco._targetTrainSpeed) + " (current: " + String(loco._currentTrainSpeed) + ", max: " + String(loco._maxTrainSpeed) + ")");
    return;
  }

  // Check for fn message
  element = xmlDocument.FirstChildElement("fn");
  if (element != NULL) {
    mcLog("<fn> node found. Processing fn message...");

    // -> process fn (function) message

    // query id attribute. This is the loco id.
    // The id is a mandatory field. If not found, the message is discarded.
    // Nevertheless, the id has no effect on the controller behaviour. Only the "addr" attribute is relevant for checking if the message is for this controller - see below.
    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      mcLog("id attribute not found or wrong type.");
      return;
    }
    mcLog("function id: " + String(rr_id));

    // query addr attribute. This is the MattzoController id.
    // If this does not equal the LOCO_ADDRESS of this controller, the message is disregarded.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog("addr attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("addr: " + String(rr_addr));

    int locoIndex = getMattzoLocoIndexByLocoAddress(rr_addr);
    if (locoIndex < 0) {
      mcLog("Message disregarded, as this controller does not handle train " + String(rr_addr));
      return;
    }
    MattzoLoco& loco = myLocos[locoIndex];
    mcLog("Consuming message for train " + loco.getNiceName());

    // query fnchanged attribute. This is information which function shall be set.
    int rr_functionNo;
    if (element->QueryIntAttribute("fnchanged", &rr_functionNo) != XML_SUCCESS) {
      mcLog("fnchanged attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("fnchanged: " + String(rr_functionNo));

    if (rr_functionNo < 1 || rr_functionNo > NUM_FUNCTIONS) {
      mcLog("fnchanged out of range. Message disregarded.");
      return;
    }
    int functionPinId = rr_functionNo - 1;
    mcLog("Function PIN Id: " + String(functionPinId));

    // Check if the function is associated with the loco
    if (FUNCTION_PIN_LOCO_ADDRESS[functionPinId] != loco._locoAddress) {
      mcLog("Function PIN is associated with loco " + String(FUNCTION_PIN_LOCO_ADDRESS[functionPinId]) + ", not with " + String(loco._locoAddress) + ". Disregarding message.");
      return;
    }

    // query fnchangedstate attribute. This is value if the function shall be set on or off
    const char* rr_state = "xxxxxx";  // expected values are "true" or "false"
    if (element->QueryStringAttribute("fnchangedstate", &rr_state) != XML_SUCCESS) {
      mcLog("fnchangedstate attribute not found or wrong type.");
      return;
    }
    mcLog("fnchangedstate (raw): " + String(rr_state));
    if (strcmp(rr_state, "true") == 0) {
      mcLog("fnchangedstate: true");
      functionCommand[functionPinId] = true;
    }
    else if (strcmp(rr_state, "false") == 0) {
      mcLog("fnchangedstate: false");
      functionCommand[functionPinId] = false;
    }
    else {
      mcLog("unknown fnchangedstate value - disregarding message.");
      return;
    }

    return;
  }

  // Check for sys message
  element = xmlDocument.FirstChildElement("sys");
  if (element != NULL) {
    mcLog("<sys> node found. Processing sys message...");

    const char* rr_cmd = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";

    // query cmd attribute. This is the system message type.
    if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
      mcLog("cmd attribute not found or wrong type.");
      return;
    }

    String rr_cmd_s = String(rr_cmd);
    mcLog("rocrail system command: " + String(rr_cmd_s));

    // Upon receiving "stop", "ebreak" or "shutdown" system command from Rocrail, the global emergency break flag is set. Train will stop immediately.
    // Upon receiving "go" command, the emergency break flag is be released (i.e. pressing the light bulb in Rocview).

    if (rr_cmd_s == "ebreak" || rr_cmd_s == "stop" || rr_cmd_s == "shutdown") {
      mcLog("received ebreak, stop or shutdown command. Stopping train.");
      ebreak = true;
    }
    else if (rr_cmd_s == "go") {
      mcLog("received go command. Releasing emergency break.");
      ebreak = false;
    }
    else {
      mcLog("received other system command, disregarded.");
    }
    return;
  }

  mcLog("Unknown message, disregarded.");
}

// set powered up motor speed
void setTrainSpeed(int newTrainSpeed, int locoIndex) {
  MattzoLoco& loco = myLocos[locoIndex];

  // set motor power
  int power = map(newTrainSpeed, 0, loco._maxTrainSpeed, 0, MattzoPUHub::MAX_PU_POWER * loco._maxTrainSpeed / 100);
  mcLog("Setting motor speed: " + String(newTrainSpeed) + " (power: " + String(power) + ")");
  for (int i = 0; i < NUM_HUBS; i++) {
    myHubs[i].setMotorSpeed(power, loco._locoAddress);
  }
  mcLog("Motor speed set to: " + String(newTrainSpeed));

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
  for (int locoIndex; locoIndex < NUM_LOCOS; locoIndex++) {
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
        boolean accelerateFlag = abs(loco._currentTrainSpeed) < abs(loco._targetTrainSpeed) && (loco._currentTrainSpeed * loco._targetTrainSpeed > 0);
        int step = accelerateFlag ? loco._accelerateStep : loco._brakeStep;
  
        int nextSpeed;
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
        mcLog("Light event stop");
        // switch all functions off
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        functionCommand[i] = false;
        break;
      case LightEventType::FORWARD:
        mcLog("Light event forward");
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        // functionCommand[i] = (i % 2) == 0;
        // functionCommand[i] = true;
        functionCommand[i] = i <= 1;
        break;
      case LightEventType::REVERSE:
        mcLog("Light event reverse");
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        // functionCommand[i] = (i % 2) == 1;
        // functionCommand[i] = true;
        functionCommand[i] = i >= 1;
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
      mcLog("Flipping function " + String(i + 1) + " to " + String(onOff));
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
  // return;  // comment out to display debug info every 10 seconds

  if (millis() - timeLastHubReport < 10000) return;

  mcLog("*** DEBUG-INFO ***");
  for (int i = 0; i < NUM_LOCOS; i++) {
    mcLog("Loco " + String(i) + ": " + myLocos[i].getNiceName() + ". Target speed: " + String(myLocos[i]._targetTrainSpeed) + ", current speed: " + String(myLocos[i]._currentTrainSpeed) + ", max speed: " + String(myLocos[i]._maxTrainSpeed));
  }
  mcLog("next hub to initialize (if any): " + String(initializedHub));
  for (int i = 0; i < NUM_HUBS; i++) {
    mcLog("Hub " + String(i) + ": macAddress=" + myHubs[i]._macAddress + ": isConnecting()=" + String(myHubs[i].isConnecting()) + " isConnected()=" + String(myHubs[i].isConnected()) + " status=" + String(myHubs[i].lastKnownConnectionStatus));
  }
  timeLastHubReport = millis();
  mcLog("******************");
}
