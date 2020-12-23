// MattzoTrainController for Powered Up Firmware
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// ****************************************
// TARGET-PLATTFORM for this sketch: ESP-32
// ****************************************

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

// MattzoBricks library files
#include "MTC4PU_Configuration.h"     // this file should be placed in the same folder
#include "MattzoController_Library.h" // this file needs to be placed in the Arduino library folder


/* Functions */
const int NUM_FUNCTIONS = 1;          // While theoretically possible to switch two lights on a Powered Up hub independently, this is not supported. So the number of "functions" is one.
bool functionCommand[NUM_FUNCTIONS];  // Desired state of a function
bool functionState[NUM_FUNCTIONS];    // Actual state of a function

/* Legoino Library */
const int NUM_HUBS = 8;  // Number of connected hubs

String discoveryHubAddress;   // Will be send to MQTT as soon connection to MQTT established.
int initializedHub = -1;   // The presently initialized hub. Only one hub can be initialized at the moment. :-(
Lpf2Hub myHubs[NUM_HUBS];  // Objects for Powered Up Hubs for the Legoino library

// Main hub array
//              |-- number of Hubs
//              |
//              |         |-- { "name", "address", "connectionStatus", "motorPorts", "lightPorts"}
//              |         |
//              v         v
char* myHubData[NUM_HUBS][5] =
{
 {"ICE1", "90:84:2b:16:15:f8", "false", "A", "B"},
 {"ICE2", "90:84:2b:17:e9:4c", "false", "A", ""},
 {"EST1", "90:84:2b:18:f2:52", "false", "A", ""},
 {"EST2", "90:84:2b:18:f7:75", "false", "A", ""},
 {"BROCO", "90:84:2b:0f:ac:c7", "false", "B", ""},
 {"GRECO", "90:84:2b:21:71:46", "false", "A", ""},
 {"BANAAN1", "90:84:2b:01:20:f8", "false", "A", ""},
 {"BANAAN2", "90:84:2b:00:5d:bb", "false", "A", ""}
};
// {"ICE1", "90:84:2b:16:15:f8", "false", "A", "B"}
// {"ICE2", "90:84:2b:17:e9:4c", "false", "A", ""}
// {"BROCO", "90:84:2b:0f:ac:c7", "false", "B", ""}
// {"GRECO", "90:84:2b:21:71:46", "false", "A", ""}
// {"EST1", "90:84:2b:18:f2:52", "false", "A", ""}
// {"EST2", "90:84:2b:18:f7:75", "false", "A", ""}
// {"BANAAN1", "90:84:2b:01:20:f8", "false", "A", ""}
// {"BANAAN2", "90:84:2b:00:5d:bb", "false", "A", ""}

// Constants the different values of the second dimension of the hub-Array (the first dimension is the hub itself)
const int HUB_NAME = 0; // name of the hub
const int HUB_ADDRESS = 1; // MAC address
const int HUB_STATUS = 2; // connectionstatus of the hub ("true" = connected, "false" = not connected). Must be "false" in the configuration (start-up value).
const int HUB_MOTORPORT = 3; // indicates which ports have motors attached including turning direction (small letter -> reverse)
                                // Allowed options: "", "A", "a", "B", "b", "AB", "Ab", "aB", "ab" (BA also works and equals AB etc.)
const int HUB_LIGHTPORT = 4; // indicates which ports have lights attached
                                // Allowed options: "", "A", "B", "AB""

const int PU_SCAN_DURATION = 1000;  // time the controller tries to connect to a Hub (in ms)


/* Send battery level  */
const int SEND_BATTERYLEVEL_INTERVAL = 60000; // interval for sending battery level in milliseconds (60000 = 60 seconds)
unsigned long lastBatteryLevelMsg = millis();    // time of the last sent battery level
int nextBatteryLevelReportingHub = 0;            // index of the last hubs for which a battery report was requested

// Motor acceleration parameters
const int ACCELERATION_INTERVAL = 100;       // pause between individual speed adjustments in milliseconds
const int ACCELERATE_STEP = 1;               // acceleration increment for a single acceleration step
const int BRAKE_STEP = 2;                    // brake decrement for a single braking step

// Speed variables
int currentTrainSpeed = 0;                   // current speed of this train
int targetTrainSpeed = 0;                    // Target speed of this train
int maxTrainSpeed = 0;                       // Maximum speed of this train as configured in Rocrail
unsigned long lastAccelerate = millis();     // time of the last speed adjustment

// Global emergency brake flag.
boolean ebreak = false;



void setup() {
  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    functionCommand[i] = false;
    functionState[i] = false;
  }

  // load config from EEPROM, initialize Wifi, MQTT etc.
  setupMattzoController();

  // discover new Powered Up Hubs (just to display MAC address)
  // discoverPoweredUpHubs();
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

void sendMQTTBatteryLevel() {
  if (millis() - lastBatteryLevelMsg >= SEND_BATTERYLEVEL_INTERVAL / NUM_HUBS) {
    lastBatteryLevelMsg = millis();

    if (myHubs[nextBatteryLevelReportingHub].isConnected()) {
      mcLog("Requesting battery level for hub " + String(nextBatteryLevelReportingHub));
      myHubs[nextBatteryLevelReportingHub].activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
      delay(10);
    }

    nextBatteryLevelReportingHub = (++nextBatteryLevelReportingHub) % NUM_HUBS;
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
    if (rr_addr != LOCO_ADDRESS) {
      mcLog("Message disgarded, as it is not for me, but for MattzoController No. " + String(rr_addr));
      return;
    }

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
    targetTrainSpeed = rr_v * dir;
    maxTrainSpeed = rr_vmax;
    mcLog("Message parsing complete, target speed set to " + String(targetTrainSpeed) + " (current: " + String(currentTrainSpeed) + ", max: " + String(maxTrainSpeed) + ")");
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
    if (rr_addr != LOCO_ADDRESS) {
      mcLog("Message disgarded, as it is not for me, but for MattzoController No. " + String(rr_addr));
      return;
    }

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
void setTrainSpeed(int newTrainSpeed) {
  const int DELAY = 10;  // a small delay after setting the motor speed is required, else the call to the Legoino library will crash
  const int MAX_PU_POWER = 88;

  int power = map(newTrainSpeed, 0, maxTrainSpeed, 0, MAX_PU_POWER * maxTrainSpeed / 100);
  mcLog("Setting motor speed: " + String(newTrainSpeed) + " (power: " + String(power) + ")");

  currentTrainSpeed = newTrainSpeed;
  // Set integrated powered up hub light according to situation
  Color ledColor;
  if (currentTrainSpeed != targetTrainSpeed) {
    // accelerating / braking
    ledColor = YELLOW;
  }
  else if (currentTrainSpeed == 0) {
    // stopped
    ledColor = RED;
  }
  else {
    // travelling at target speed
    ledColor = GREEN;
  }

  for (int i = 0; i < NUM_HUBS; i++) {
    if (myHubs[i].isConnected()) {
      if (String(myHubData[i][HUB_MOTORPORT]).indexOf("A") >= 0) {
        myHubs[i].setBasicMotorSpeed((byte)PoweredUpHubPort::A, power);
        delay(DELAY);
      }
      else if (String(myHubData[i][HUB_MOTORPORT]).indexOf("a") >= 0) {
        myHubs[i].setBasicMotorSpeed((byte)PoweredUpHubPort::A, -power);
        delay(DELAY);
      }

      if (String(myHubData[i][HUB_MOTORPORT]).indexOf("B") >= 0) {
        myHubs[i].setBasicMotorSpeed((byte)PoweredUpHubPort::B, power);
        delay(DELAY);
      }
      else if (String(myHubData[i][HUB_MOTORPORT]).indexOf("b") >= 0) {
        myHubs[i].setBasicMotorSpeed((byte)PoweredUpHubPort::B, -power);
        delay(DELAY);
      }

      // Set integrated powered up hub light
      myHubs[i].setLedColor(ledColor);
      delay(DELAY);
    }
  }

  mcLog("Motor speed set to: " + String(newTrainSpeed));
}


// gently adapt train speed (increase/decrease slowly)
void accelerateTrainSpeed() {
  if (ebreak) {
    // emergency break pulled and train moving -> stop train immediately
    if (currentTrainSpeed != 0) {
      setTrainSpeed(0);
    }
  }
  else if (currentTrainSpeed != targetTrainSpeed) {
    if (targetTrainSpeed == 0) {
      // stop -> execute immediately
      setTrainSpeed(0);
    }
    else if (millis() - lastAccelerate >= ACCELERATION_INTERVAL) {
      lastAccelerate = millis();

      // determine if trains accelerates or brakes
      boolean accelerateFlag = abs(currentTrainSpeed) < abs(targetTrainSpeed) && (currentTrainSpeed * targetTrainSpeed > 0);
      int step = accelerateFlag ? ACCELERATE_STEP : BRAKE_STEP;

      int nextSpeed;
      // accelerate / brake gently
      if (currentTrainSpeed < targetTrainSpeed) {
        nextSpeed = min(currentTrainSpeed + step, targetTrainSpeed);
      }
      else {
        nextSpeed = max(currentTrainSpeed - step, targetTrainSpeed);
      }
      setTrainSpeed(nextSpeed);
    }
  }
}

// switch lights on or off
void setLights() {
  const int DELAY = 10;  // a small delay after setting the light intensity on the PU hub is required, else the call to the Legoino library will crash

  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    bool onOff = functionCommand[i];

    if (ebreak) {
      // override function state on ebreak (alternate lights on/off every 500ms)
      long phase = (millis() / 500) % 2;
      onOff = (phase + i) % 2 == 0;
    }

    if (functionState[i] != onOff) {
      functionState[i] = onOff;
      mcLog("Flipping function " + String(i + 1));

      int lightPower = onOff ? 100 : 0;

      for (int h = 0; h < NUM_HUBS; h++) {
        if (myHubs[h].isConnected()) {
          if (String(myHubData[h][HUB_LIGHTPORT]).indexOf("A") >= 0) {
            myHubs[i].setBasicMotorSpeed((byte)PoweredUpHubPort::A, lightPower);
            delay(DELAY);
          }
          else if (String(myHubData[h][HUB_LIGHTPORT]).indexOf("B") >= 0) {
            myHubs[i].setBasicMotorSpeed((byte)PoweredUpHubPort::B, lightPower);
            delay(DELAY);
          }
        }
      }
    }
  }
}

// if a Powered Hub was discovered on start - up, transmit once after initial MQTT connection.
void transmitDiscoveredHub() {
  if (discoveryHubAddress.length() > 0) {
    if (getConnectionStatus() == MCConnectionStatus::CONNECTED) {
      sendMQTTMessage("roc2bricks/discovery", "Powered Up Hub discovered: " + discoveryHubAddress);
      discoveryHubAddress = "";
    }
  }
}

// TODO: call the callback does not work yet... needs investigation.
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

void loop() {
  loopMattzoController();
  transmitDiscoveredHub();
  reconnectHUB();
  accelerateTrainSpeed();
  setLights();
  sendMQTTBatteryLevel();
}

void discoverPoweredUpHubs() {
  // Just after booting, the ESP-32 checks if there is a Powered Up hub that is ready to connect
  // Purpose is just to read out its MAC address at publish it via MQTT.
  // The MAC address is then known and can be used in the configuration of the MattzoController (recompiling the source code is required)

  Lpf2Hub discoveryHub;  // Hub used for discovering new PU hubs

  mcLog("Discovering Powered Up Hubs...");
  discoveryHub.init();
  unsigned long timeStart = millis();
  while (millis() - timeStart < 10000) {  // wait 10 seconds for a hub to connect
    if (discoveryHub.isConnecting()) {
      discoveryHubAddress = discoveryHub.getHubAddress().toString().c_str();
      mcLog("Powered Up Hub found. MAC Address: " + discoveryHubAddress);
      return;
    }
  }
  mcLog("No Powered Up Hub found. Discovering new Powered Up hubs terminated.");
}



void initPoweredUpHub(int hubIndex) {
  mcLog("Initializing hub " + String(hubIndex) + "...");
  myHubs[hubIndex].init(myHubData[hubIndex][HUB_ADDRESS], PU_SCAN_DURATION);
  delay(10);
}

unsigned long timeLastHubReport = 0;

void reconnectHUB() {
  // check for hubs ready for connection (is connecting, but not connected) -> connect
  for (int i = 0; i < NUM_HUBS; i++) {
    if (!myHubs[i].isConnected()) {
      // has this hub been initialized?
      if (i != initializedHub) {
        initPoweredUpHub(i);
        initializedHub = i;
      }

      // hub connecting -> connect!
      if (myHubs[i].isConnecting()) {
        // Connect to hub
        mcLog("Connecting to hub " + String(i) + "...");
        myHubs[i].connectHub();
  
        if (myHubs[i].isConnected()) {
          // Send "connected" message
          mcLog("Connected to hub " + String(i) + ".");
          sendMQTTMessage("roc2bricks/connectionStatus", String(myHubData[i][HUB_ADDRESS]) + " connected");
          myHubData[i][HUB_STATUS] = "true";
        }
        else {
          mcLog("Connection attempt to hub " + String(i) + " refused.");
          initPoweredUpHub(i);
        }
      } else {
        if (String(myHubData[i][HUB_STATUS]) == String("true")) {
          // Send "connection lost" message
          mcLog("Hub " + String(i) + " disconnected.");
          sendMQTTMessage("roc2bricks/connectionStatus", String(myHubData[i][HUB_ADDRESS]) + " disconnected");
          myHubData[i][HUB_STATUS] = "false";
          initPoweredUpHub(i);
        }
      }

      // if unconnected hub found, do not process any other hubs at this time.
      break;
    }
  }

  // Debugging code... may be removed.
  return;

  if (millis() - timeLastHubReport < 10000) return;

  mcLog("***");
  mcLog("init hub: " + String(initializedHub));
  for (int i = 0; i < NUM_HUBS; i++) {
    mcLog("Hub " + String(i) + ": isConnecting()=" + String(myHubs[i].isConnecting()) + " isConnected()=" + String(myHubs[i].isConnected()) + " status=" + String(myHubData[i][HUB_STATUS]));
  }
  timeLastHubReport = millis();
}
