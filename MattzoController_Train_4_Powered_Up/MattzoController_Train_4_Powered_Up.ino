// MattzoTrainController for Powered Up Firmware
// Author: Dr. Matthias Runte
// Libraries can be downloaded easily from within the Arduino IDE using the library manager.
// TinyXML2 must be downloaded from https://github.com/leethomason/tinyxml2 (required files: tinyxml2.cpp, tinyxml2.h)

// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// Built-in libraries of the Arduino IDE
#include <EEPROM.h>        // EEPROM library
#include <WiFi.h>          // WiFi library

// PubSubClient library by Nick O'Leary
// Install via the built-in Library Manager of the Arduino IDE
// Tested with Version V2.8.0
#include <PubSubClient.h>

// TinyXML2
// Download from https://github.com/leethomason/tinyxml2
// These files need to be placed in the Arduino library folder:
//   "tinyxml2.cpp"
//   "tinyxml2.h".
// Tested with version of date 2020-11-21.
#include <tinyxml2.h>

// Legoino library by Cornelius Munz
// Install via the built-in Library Manager of the Arduino IDE
// Tested with:
//   Version V1.0.2 of the "Legoino" library
//   Version V1.0.2 of the required dependent library "NimBLE-Arduino"
#include "Lpf2Hub.h"

// This file is supplied with the MattzoBricks Firmware package and needs to be placed in the Arduino library folder
#include <MattzoController_Network_Configuration.h>

using namespace tinyxml2;

/* MattzoController specifics */
String eepromIDString = "MattzoTrainController4PU";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 24;              // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerNo;  // controllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 16383;

/* MQTT */
String mqttClientName;                                   // Name of the MQTT client (me) with which messages are sent
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

/* Functions */
const int NUM_FUNCTIONS = 1;          // While theoretically possible to switch two lights on a Powered Up hub independently, this is not supported. So the number of "functions" is one.
bool functionCommand[NUM_FUNCTIONS];  // Desired state of a function
bool functionState[NUM_FUNCTIONS];    // Actual state of a function

/* Legoino Library */
const int NUM_HUBS = 2;  // Number of connected hubs
Lpf2Hub myHubs[NUM_HUBS];  // Objects for Powered Up Hubs for the Legoino library

// Main hub array
//              |-- number of Hubs
//              |
//              |         |-- { "name", "address", "connectionStatus", "motorPorts", "lightPorts"}
//              |         |
//              v         v
char* myHubData[NUM_HUBS][5]=
{
  {"ICE1", "90:84:2b:16:15:f8", "false", "A", "B"},
  {"ICE2", "90:84:2b:17:e9:4c", "false", "A", ""}
};
// {"ICE1", "90:84:2b:16:15:f8", "false", "A", "B"}
// {"ICE2", "90:84:2b:17:e9:4c", "false", "A", ""}
// {"Crocodile", "90:84:2b:0f:ac:c7", "false", "A", ""}

// Constants the four different Values of the second dimension of the hub-Array (the first dimension is the hub itself)
const int HUB_NAME         = 0; // name of the hub
const int HUB_ADDRESS      = 1; // MAC address
const int HUB_STATUS       = 2; // connectionstatus of the hub ("true" = connected, "false" = not connected). Must be "false" in the configuration (start-up value).
const int HUB_MOTORPORT    = 3; // indicates which ports have motors attached including turning direction (small letter -> reverse)
                                // Allowed options: "", "A", "a", "B", "b", "AB", "Ab", "aB", "ab" (BA also works and equals AB etc.)
const int HUB_LIGHTPORT    = 4; // indicates which ports have lights attached
                                // Allowed options: "", "A", "B", "AB""

// Legoino Constants (god knows why we have to do this in the script USING the library...)
byte hubPortA = (byte)PoweredUpHubPort::A;
byte hubPortB = (byte)PoweredUpHubPort::B;


/* Send battery level  */
const int SEND_BATTERYLEVEL_INTERVAL = 60000; // interval for sending battery level in milliseconds
unsigned long lastBatteryLevelMsg = millis();    // time of the last sent battery level

// Motor acceleration parameters
const int ACCELERATION_INTERVAL = 100;       // pause between individual speed adjustments in milliseconds
const int ACCELERATE_STEP = 1;               // acceleration increment for a single acceleration step
const int BRAKE_STEP = 2;                    // brake decrement for a single braking step
int currentTrainSpeed = 0;                   // current speed of this train
int targetTrainSpeed = 0;                    // Target speed of this train
int maxTrainSpeed = 0;                       // Maximum speed of this train as configured in Rocrail
unsigned long lastAccelerate = millis();     // time of the last speed adjustment

// ebreak
boolean ebreak = false;   // Global emergency break flag.

// Wifi and MQTT objects
WiFiClient espClient;
PubSubClient client(espClient);


void setup() {
  Serial.begin(115200);
  randomSeed(ESP.getCycleCount());
  Serial.println("MattzoController booting...");

  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    functionCommand[i] = false;
  }

  initMattzoController();
  initWIFI();
  initMQTT();

  // initPoweredUpHubs();
}

void initMattzoController() {
  int i;
  int controllerIDHiByte;
  int controllerIDLowByte;

  // set-up EEPROM read/write operations
  EEPROM.begin(512);

  // Check if the first part of the memory is filled with the MattzoController ID string.
  // This is the case if the controller has booted before with a MattzoController firmware.
  bool idStringCheck = true;
  for (i = 0; i < eepromIDString.length(); i++) {

    char charEeprom = EEPROM.read(i);
    char charIDString = eepromIDString.charAt(i);

    if (charEeprom != charIDString) {
      idStringCheck = false;
      break;
    }
  }

  // TODO: also write / read SSID, Wifi-Password and MQTT Server from EEPROM

  int paramsStartingPosition = eepromIDString.length();
  if (idStringCheck) {
    // load controllerNo from preferences
    controllerIDHiByte  = EEPROM.read(paramsStartingPosition);
    controllerIDLowByte = EEPROM.read(paramsStartingPosition + 1);
    controllerNo = controllerIDHiByte * 256 + controllerIDLowByte;

    Serial.println("MattzoControlerId " + String(controllerNo) + " loaded from to EEPROM.");
  } else {
    // preferences not initialized yet -> initialize controller
    // this runs only a single time when starting the controller for the first time

    // Wait a bit to give the user some time to open the serial console...
    //delay (5000);
    
    Serial.println("Initializing controller preferences on first start-up ...");
    for (i = 0; i < eepromIDString.length(); i++) {
      EEPROM.write(i, eepromIDString.charAt(i));
    }

    // assign random controllerNo between 1 and 65000 and store in EEPROM
    controllerNo = random(1, MAX_CONTROLLER_ID);
    controllerIDHiByte  = controllerNo / 256;
    controllerIDLowByte = controllerNo % 256;
    EEPROM.write(paramsStartingPosition, controllerIDHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerIDLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

    Serial.println("New MattzoControlerId " + String(controllerNo) + " written to EEPROM.");
  }

  // set MQTT client name
  mqttClientName = eepromIDString + String(controllerNo);
  mqttClientName.toCharArray(mqttClientName_char, mqttClientName.length() + 1);
}

void initWIFI() {
  Serial.println("My MAC: " + WiFi.macAddress());
  //Serial.print("Connecting to SSID " + String(WIFI_SSID) + " ");
  //WiFi.setSleepMode(WIFI_NONE_SLEEP);
  reconnectWIFI();
}

void reconnectWIFI() {
  Serial.print("Connecting to WiFi, SSID " + String(WIFI_SSID) + " ");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    // TODO: Support WPS! Store found Wifi network found via WPS in EEPROM and use next time!
/*
    // please see here: https://www.arduino.cc/en/Reference/WiFiStatus
    if (WiFi.status() == WL_IDLE_STATUS)          {Serial.println("WL_IDLE_STATUS");}
    else if (WiFi.status() == WL_CONNECTED)       {Serial.println("WL_CONNECTED ");}
    else if (WiFi.status() == WL_NO_SSID_AVAIL)   {Serial.println("WL_NO_SSID_AVAIL");}
    else if (WiFi.status() == WL_CONNECT_FAILED)  {Serial.println("WL_CONNECT_FAILED");}
    else if (WiFi.status() == WL_CONNECTION_LOST) {Serial.println("WL_CONNECTION_LOST");}
    else if (WiFi.status() == WL_DISCONNECTED)    {Serial.println("WL_DISCONNECTED");}
    else {Serial.println("unknown status: " + String(WiFi.status()));}
*/
  }

  Serial.println();
  Serial.print("Connected to WiFi, SSID " + String(WIFI_SSID) + ", as ");
  Serial.println(WiFi.localIP());
}

void initMQTT() {
  client.setServer(MQTT_BROKER_IP, 1883);
  client.setCallback(callbackMQTT);
  client.setBufferSize(2048);
  client.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);   // keep alive interval
}

void reconnectMQTT() {
  while (!client.connected()) {
      Serial.println("Reconnecting MQTT...");

      String lastWillMessage = String(mqttClientName_char) + " " + "last will and testament";
      char lastWillMessage_char[lastWillMessage.length() + 1];
      lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

      if (!client.connect(mqttClientName_char, "roc2bricks/lastWill", 0, false, lastWillMessage_char)) {
        Serial.print("Failed, rc=");
        Serial.print(client.state());
        Serial.println(". Retrying in 5 seconds...");
        delay(5000);
      }
  }
  client.subscribe("rocrail/service/command");
  Serial.println("MQTT connected, listening on topic [rocrail/service/command].");
}

void sendMQTTMessage(String topic, String message) {
  const int MAX_MQTT_TOPIC_SIZE = 255;
  const int MAX_MQTT_MESSAGE_SIZE = 255;
  char topic_char[MAX_MQTT_TOPIC_SIZE];
  char message_char[MAX_MQTT_MESSAGE_SIZE];

  if (topic.length() + 1 > MAX_MQTT_TOPIC_SIZE) {
    Serial.println("ERROR: MQTT topic string too long - message not sent.");
    return;
  }

  message = String(mqttClientName_char) + " " + message;
  if (message.length() + 1 > MAX_MQTT_MESSAGE_SIZE) {
    Serial.println("ERROR: MQTT message string too long - message not sent.");
    return;
  }

  topic.toCharArray(topic_char, topic.length() + 1);
  message.toCharArray(message_char, message.length() + 1);
  
  Serial.println("sending mqtt: " + topic + " " + message);
  client.publish(topic_char, message_char);
}

void sendMQTTBatteryLevel(){
  if (millis() - lastBatteryLevelMsg >= SEND_BATTERYLEVEL_INTERVAL) {
    lastBatteryLevelMsg = millis();

    for (int i = 0; i < NUM_HUBS; i++){
      if (myHubs[i].isConnected()) {
        Serial.println("(deparecated) requesting battery level for hub " + i);
      }
    }
  }
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length+1];
  for (int i = 0; i < length; i++) {
      msg[i] = (char)payload[i];
  }
  msg[length] = '\0';
  Serial.print(msg);

  XMLDocument xmlDocument;
  if(xmlDocument.Parse(msg)!= XML_SUCCESS){
    Serial.println("Error parsing");
    return;
  }

  Serial.println("Parsing XML successful");

  const char * rr_id = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";
  int rr_addr = 0;

  // check for lc message
  XMLElement * element = xmlDocument.FirstChildElement("lc");
  if (element != NULL) {
    Serial.println("<lc> node found. Processing loco message...");

    // -> process lc (loco) message

    // query id attribute. This is the loco id.
    // The id is a mandatory field. If not found, the message is discarded.

    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      Serial.println("id attribute not found or wrong type.");
      return;
    }
    Serial.println("loco id: " + String(rr_id));
  
    // query addr attribute. This is the MattzoController id.
    // If this does not equal the controllerNo of this controller, the message is disregarded.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      Serial.println("addr attribute not found or wrong type. Message disregarded.");
      return;
    }
    Serial.println("addr: " + String(rr_addr));
    if (rr_addr != controllerNo) {
      Serial.println("Message disgarded, as it is not for me, but for MattzoController No. " + String(rr_addr));
      return;
    }

    // query dir attribute. This is direction information for the loco (forward, backward)
    const char * rr_dir = "xxxxxx";  // expected values are "true" or "false"
    int dir;
    if (element->QueryStringAttribute("dir", &rr_dir) != XML_SUCCESS) {
      Serial.println("dir attribute not found or wrong type.");
      return;
    }
    Serial.println("dir (raw): " + String(rr_dir));
    if (strcmp(rr_dir, "true")==0) {
      Serial.println("direction: forward");
      dir = 1;
    }
    else if (strcmp(rr_dir, "false")==0) {
      Serial.println("direction: backward");
      dir = -1;
    }
    else {
      Serial.println("unknown dir value - disregarding message.");
      return;
    }

    // query V attribute. This is the speed information for the loco and ranges from 0 to V_max (see below).
    int rr_v = 0;
    if (element->QueryIntAttribute("V", &rr_v) != XML_SUCCESS) {
      Serial.println("V attribute not found or wrong type. Message disregarded.");
      return;
    }
    Serial.println("V: " + String(rr_v));

  // query V_max attribute. This is maximum speed of the loco. It must be set in the loco settings in Rocrail as percentage value.
  // The V_max attribute is required to map to loco speed from rocrail to a power setting in the MattzoController.
  int rr_vmax = 0;
  if (element->QueryIntAttribute("V_max", &rr_vmax) != XML_SUCCESS) {
    Serial.println("V_max attribute not found or wrong type. Message disregarded.");
  return;
  }
  Serial.println("V_max: " + String(rr_vmax));

  // set target train speed
  targetTrainSpeed = rr_v * dir;
  maxTrainSpeed = rr_vmax;
  Serial.println("Message parsing complete, target speed set to " + String(targetTrainSpeed) + " (current: " + String(currentTrainSpeed) + ", max: " + String(maxTrainSpeed) + ")");
    return;
  }

  // Check for fn message
  element = xmlDocument.FirstChildElement("fn");
  if (element != NULL) {
    Serial.println("<fn> node found. Processing fn message...");

    // -> process fn (function) message

    // query id attribute. This is the loco id.
    // The id is a mandatory field. If not found, the message is discarded.
    // Nevertheless, the id has no effect on the controller behaviour. Only the "addr" attribute is relevant for checking if the message is for this controller - see below.
    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      Serial.println("id attribute not found or wrong type.");
      return;
    }
    Serial.println("function id: " + String(rr_id));
  
    // query addr attribute. This is the MattzoController id.
    // If this does not equal the ControllerNo of this controller, the message is disregarded.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      Serial.println("addr attribute not found or wrong type. Message disregarded.");
      return;
    }
    Serial.println("addr: " + String(rr_addr));
    if (rr_addr != controllerNo) {
      Serial.println("Message disgarded, as it is not for me, but for MattzoController No. " + String(rr_addr));
      return;
    }

    // query fnchanged attribute. This is information which function shall be set.
    int rr_functionNo;
    if (element->QueryIntAttribute("fnchanged", &rr_functionNo) != XML_SUCCESS) {
      Serial.println("fnchanged attribute not found or wrong type. Message disregarded.");
      return;
    }
    Serial.println("fnchanged: " + String(rr_functionNo));

    if (rr_functionNo < 1 || rr_functionNo > NUM_FUNCTIONS) {
      Serial.println("fnchanged out of range. Message disregarded.");
      return;
    }
    int functionPinId = rr_functionNo - 1;
    Serial.println("Function PIN Id: " + String(functionPinId));

    // query fnchangedstate attribute. This is value if the function shall be set on or off
    const char * rr_state = "xxxxxx";  // expected values are "true" or "false"
    if (element->QueryStringAttribute("fnchangedstate", &rr_state) != XML_SUCCESS) {
      Serial.println("fnchangedstate attribute not found or wrong type.");
      return;
    }
    Serial.println("fnchangedstate (raw): " + String(rr_state));
    if (strcmp(rr_state, "true")==0) {
      Serial.println("fnchangedstate: true");
      functionCommand[functionPinId] = true;
    }
    else if (strcmp(rr_state, "false")==0) {
      Serial.println("fnchangedstate: false");
      functionCommand[functionPinId] = false;
    }
    else {
      Serial.println("unknown fnchangedstate value - disregarding message.");
      return;
    }

    return;
  }

  // Check for sys message
  element = xmlDocument.FirstChildElement("sys");
  if (element != NULL) {
    Serial.println("<sys> node found. Processing sys message...");

    const char * rr_cmd = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";

    // query cmd attribute. This is the system message type.
    if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
      Serial.println("cmd attribute not found or wrong type.");
      return;
    }

    String rr_cmd_s = String(rr_cmd);
    Serial.println("rocrail system command: " + String(rr_cmd_s));

    // Upon receiving "stop", "ebreak" or "shutdown" system command from Rocrail, the global emergency break flag is set. Train will stop immediately.
    // Upon receiving "go" command, the emergency break flag is be released (i.e. pressing the light bulb in Rocview).

    if (rr_cmd_s == "ebreak" || rr_cmd_s == "stop" || rr_cmd_s == "shutdown") {
      Serial.println("received ebreak, stop or shutdown command. Stopping train.");
      ebreak = true;
    } else if (rr_cmd_s == "go") {
      Serial.println("received go command. Releasing emergency break.");
      ebreak = false;
    } else {
      Serial.println("received other system command, disregarded.");
    }
    return;
  }

  Serial.println("Unknown message, disregarded.");
}


void initPoweredUpHubs() {
  // initialize all Powered Up hubs that are expected to connect to this controller
  for (int i = 0; i < NUM_HUBS; i++) {
    myHubs[i].init(myHubData[i][HUB_ADDRESS]);
    delay(100);
    myHubs[i].activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);
    delay(100);
    myHubs[i].activateHubPropertyUpdate(HubPropertyReference::BUTTON, hubPropertyChangeCallback);
    delay(100);
  }
}

void reconnectHUB(){
  int i;

  // init disconnected hubs from list and send "disconnected" information if lost
  for (i = 0; i < NUM_HUBS; i++) {
    if (!myHubs[i].isConnected() && !myHubs[i].isConnecting()) {
      if (String(myHubData[i][HUB_STATUS]) == String("true")) {
        // Send "connection lost" message
        sendMQTTMessage("roc2bricks/connectionStatus", String(myHubData[i][HUB_ADDRESS]) + " disconnected");
        myHubData[i][HUB_STATUS] = "false";
      }

      myHubs[i].init(myHubData[i][HUB_ADDRESS]);
    }
  }

  // connect to the hubs from the list and send "connected" information
  for (i = 0; i < NUM_HUBS; i++) {
    if (myHubs[i].isConnecting() && !myHubs[i].isConnected()) {
      // Connect to hub
      myHubs[i].connectHub();

      if (myHubs[i].isConnected()) {
        // Send "connected" message
        sendMQTTMessage("roc2bricks/connectionStatus", String(myHubData[i][HUB_ADDRESS]) + " connected");
        myHubData[i][HUB_STATUS] = "true";
      } else {
        myHubData[i][HUB_STATUS] = "false";
      }
    }
  }
}

void hubPropertyChangeCallback(void *hub, HubPropertyReference hubProperty, uint8_t *pData)
{
  Lpf2Hub *myHub = (Lpf2Hub *)hub;
  String hubAddress = myHub->getHubAddress().toString().c_str();
  Serial.print("HubAddress: ");
  Serial.println(hubAddress);

  Serial.print("HubProperty: ");
  Serial.println((byte)hubProperty, HEX);
  
  if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE)
  {
    Serial.print("BatteryLevel: ");
    Serial.println(myHub->parseBatteryLevel(pData), DEC);
    sendMQTTMessage("roc2bricks/battery", hubAddress + " " + myHub->parseBatteryLevel(pData));
    return;
  }

  if (hubProperty == HubPropertyReference::BUTTON)
  {
    Serial.print("Button: ");
    Serial.println((byte)myHub->parseHubButton(pData), HEX);
    sendMQTTMessage("roc2bricks/button", hubAddress + " button pressed.");
    return;
  }
}



// set powered up motor speed
void setTrainSpeed(int newTrainSpeed) {
  const int DELAY = 10;  // a small delay after setting the motor speed is required, else the call to the Legoino library will crash
  const int MAX_PU_POWER = 88;

  int power = map(newTrainSpeed, 0, maxTrainSpeed, 0, MAX_PU_POWER * maxTrainSpeed / 100);
  Serial.println("Setting motor speed: " + String(newTrainSpeed) + " (power: " + String(power) + ")");

  currentTrainSpeed = newTrainSpeed;
  // Set integrated powered up hub light according to situation
  Color ledColor;
  if (currentTrainSpeed != targetTrainSpeed) {
    // accelerating / braking
    ledColor = YELLOW;
  } else if (currentTrainSpeed == 0) {
    // stopped
    ledColor = RED;
  } else {
    // travelling at target speed
    ledColor = GREEN;
  }

  for (int i = 0; i < NUM_HUBS; i++){
    if (myHubs[i].isConnected()) {
      if (String(myHubData[i][HUB_MOTORPORT]).indexOf("A") >= 0) {
        myHubs[i].setBasicMotorSpeed(hubPortA, power);
        delay(DELAY);
      } else if (String(myHubData[i][HUB_MOTORPORT]).indexOf("a") >= 0) {
        myHubs[i].setBasicMotorSpeed(hubPortA, -power);
        delay(DELAY);
      }

      if (String(myHubData[i][HUB_MOTORPORT]).indexOf("B") >= 0) {
        myHubs[i].setBasicMotorSpeed(hubPortB, power);
        delay(DELAY);
      } else if (String(myHubData[i][HUB_MOTORPORT]).indexOf("b") >= 0) {
        myHubs[i].setBasicMotorSpeed(hubPortB, -power);
        delay(DELAY);
      }

      // Set integrated powered up hub light
      myHubs[i].setLedColor(ledColor);
      delay(DELAY);
    }
  }

  Serial.println("Motor speed set to: " + String(newTrainSpeed));
}


// gently adapt train speed (increase/decrease slowly)
void accelerateTrainSpeed() {
  if (ebreak) {
    // emergency break pulled and train moving -> stop train immediately
    if (currentTrainSpeed != 0) {
      setTrainSpeed(0);
    }
  } else if (currentTrainSpeed != targetTrainSpeed) {
    if (targetTrainSpeed == 0){
      // stop -> execute immediately
      setTrainSpeed(0);
    } else if (millis() - lastAccelerate >= ACCELERATION_INTERVAL) {
      lastAccelerate = millis();

      // determine if trains accelerates or brakes
      boolean accelerateFlag = abs(currentTrainSpeed) < abs(targetTrainSpeed) && (currentTrainSpeed * targetTrainSpeed > 0);
      int step = accelerateFlag ? ACCELERATE_STEP : BRAKE_STEP;

      int nextSpeed;
      // accelerate / brake gently
      if (currentTrainSpeed < targetTrainSpeed) {
        nextSpeed = min(currentTrainSpeed + step, targetTrainSpeed);
      } else {
        nextSpeed = max(currentTrainSpeed - step, targetTrainSpeed);
      }
      setTrainSpeed(nextSpeed);
    }
  }
}


// switch lights on or off
void setLights() {
  const int DELAY = 10;  // a small delay after setting the motor speed is required, else the call to the Legoino library will crash

  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    bool onOff = functionCommand[i];

    if (ebreak) {
      // override function state on ebreak (alternate lights on/off every 500ms)
      long phase = (millis() / 500) % 2;
      onOff = (phase + i) % 2 == 0;
    }

    if (functionState[i] != onOff) {
      functionState[i] = onOff;
      Serial.println("Flipping function " + String(i+1));

      int lightPower = onOff ? 100 : 0;

      for (int h = 0; h < NUM_HUBS; h++) {
        if (myHubs[h].isConnected()) {
          if (String(myHubData[h][HUB_LIGHTPORT]).indexOf("A") >= 0) {
            myHubs[i].setBasicMotorSpeed(hubPortA, lightPower);
            delay(DELAY);
          } else if (String(myHubData[h][HUB_LIGHTPORT]).indexOf("B") >= 0) {
            myHubs[i].setBasicMotorSpeed(hubPortB, lightPower);
            delay(DELAY);
          }
        }
      }
    }
  }
}


void loop() {
  if (WiFi.status() != WL_CONNECTED){
    reconnectWIFI();
  }

  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  reconnectHUB();

  accelerateTrainSpeed();
  setLights();

  // TODO: clean up
  // sendMQTTPing(&client, mqttClientName_char);
  // sendMQTTBatteryLevel();
}
