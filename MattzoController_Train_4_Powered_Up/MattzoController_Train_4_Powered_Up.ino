// MattzoSwitchController Firmware V0.1
// Author: Dr. Matthias Runte
// Libraries can be downloaded easily from within the Arduino IDE using the library manager.
// TinyXML2 must be downloaded from https://github.com/leethomason/tinyxml2 (required files: tinyxml2.cpp, tinyxml2.h)

// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <EEPROM.h>        // EEPROM library
#include <WiFi.h>          // WiFi library
#include <PubSubClient.h>  // MQTT library
#include <tinyxml2.h>      // https://github.com/adafruit/TinyXML
#include <PoweredUpHub.h>  // https://github.com/corneliusmunz/legoino
#include <MattzoController_Network_Configuration.h>  // this file needs to be placed in the Arduino library folder

using namespace tinyxml2;

/* MattzoController specifics */
String eepromIDString = "MattzoTrainController4PU";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 24;              // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerID;                        // controller id. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 16383;

/* MQTT */
String mqttClientName;                                   // Name of the MQTT client (me) with which messages are sent
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.


/* Timing */

/* Current time for event timing */
unsigned long currentMillis = millis();

/* Send a ping to announce that you are still alive */
const int SEND_PING_INTERVAL = 5000;   // interval for sending pings in milliseconds
unsigned long lastPing = millis();    // time of the last sent ping

/* Send the battery level  */
const int SEND_BATTERYLEVEL_INTERVAL = 60000; // interval for sending battery level in milliseconds
unsigned long lastBatteryLevelMsg = millis();    // time of the last sent battery level


/* LEGO Powered up */
// Number of connected hubs
const int NUM_HUBS = 1;

// Note: the controller will connect to hubs only - remotes are refused.
PoweredUpHub myHubs[NUM_HUBS];  // Objects for the "hubs" (A and B)

// Constants the four different Values of the second dimension of the hub-Array (the first dimension is the hub itself)
const int HUB_NAME         = 0; // name of the hub
const int HUB_ADDRESS      = 1; // MAC address
const int HUB_STATUS       = 2; // connectionstatus of the hub ("true" = connected, "false" = not connected)
const int HUB_MOTORPORT    = 3; // ports with motors (A, -A,  AB, -AB, -A-B, A-B, B, -B), a minus indicates an inverse rotation direction

// Main hub array
//              |-- number of Hubs
//              |
//              |         |-- { "name", "address", "connectionStatus", "port"}
//              |         |
//              v         v
char* myHubData[NUM_HUBS][4]=
{
  {"Crocodile", "90:84:2b:0f:ac:c7", "false", "A"}
};
//  {"ICE1", "90:84:2b:16:15:f8", "false", "B-"}


PoweredUpHub::Port hubPortA = PoweredUpHub::Port::A; // port A
PoweredUpHub::Port hubPortB = PoweredUpHub::Port::B; // port B

// Motor acceleration parameters
const int ACCELERATION_INTERVAL = 100;       // pause between individual speed adjustments in milliseconds
const int ACCELERATE_STEP = 1;               // acceleration increment for a single acceleration step
const int BRAKE_STEP = 2;                    // brake decrement for a single braking step
int currentTrainSpeed = 0;                   // current speed of this train
int targetTrainSpeed = 0;                    // Target speed of this train
int maxTrainSpeed = 0;                       // Maximum speed of this train as configured in Rocrail
unsigned long lastAccelerate = millis();     // time of the last speed adjustment

// Wifi and MQTT objects
WiFiClient espClient;
PubSubClient client(espClient);


void setup() {
  Serial.begin(115200);
  randomSeed(ESP.getCycleCount());
  Serial.println("MattzoController booting...");

  initMattzoController();
  initWIFI();
  initMQTT();

  //initPoweredUpHubs();
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
    // load controllerID from preferences
    controllerIDHiByte  = EEPROM.read(paramsStartingPosition);
    controllerIDLowByte = EEPROM.read(paramsStartingPosition + 1);
    controllerID = controllerIDHiByte * 256 + controllerIDLowByte;

  } else {
    // preferences not initialized yet -> initialize controller
    // this runs only a single time when starting the controller for the first time

    // Wait a bit to give the user some time to open the serial console...
    //delay (5000);
    
    Serial.println("Initializing controller preferences on first start-up ...");
    for (i = 0; i < eepromIDString.length(); i++) {
      EEPROM.write(i, eepromIDString.charAt(i));
    }

    // assign random controllerID between 1 and 65000 and store in EEPROM
    controllerID = random(1, MAX_CONTROLLER_ID);
    controllerIDHiByte  = controllerID / 256;
    controllerIDLowByte = controllerID % 256;
    EEPROM.write(paramsStartingPosition, controllerIDHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerIDLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

  }
  Serial.println("New controller id " + String(controllerID) + " written to EEPROM.");
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
  // set MQTT client name
  mqttClientName = eepromIDString + " " + String(controllerID);
  mqttClientName.toCharArray(mqttClientName_char, mqttClientName.length() + 1);

  client.setServer(MQTT_BROKER_IP, 1883);
  client.setCallback(callbackMQTT);
  client.setBufferSize(2048);
}

void reconnectMQTT() {
  Serial.println("MQTT connecting ...");
  while (!client.connected()) {

      char message_char[100];
      String message = String(mqttClientName_char) + " " + "last will and testament";
      message.toCharArray(message_char, message.length() + 1);

      if (!client.connect(mqttClientName_char, "roc2bricks/lastWill", 0, false, message_char)) {
      //if (!client.connect(mqttClientName_char)) {
        Serial.print("Failed, rc=");
        Serial.println(client.state());
        delay(500);
      }
  }
  client.subscribe("rocrail/service/command");
  Serial.println("MQTT connected");
}

void sendMQTTPing(){
  if (currentMillis - lastPing >= SEND_PING_INTERVAL) {
    lastPing = millis();
    Serial.println("sending ping...");
    client.publish("roc2bricks/ping", mqttClientName_char);
  }
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
  if (currentMillis - lastBatteryLevelMsg >= SEND_BATTERYLEVEL_INTERVAL) {
    lastBatteryLevelMsg = millis();

    for (int i = 0; i < NUM_HUBS; i++){
      if (myHubs[i].isConnected()) {
        Serial.println("sending battery level for hub " + i);
        sendMQTTMessage("roc2bricks/battery", String(myHubData[i][HUB_ADDRESS]) + " " + String(myHubs[i].getBatteryLevel()));
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

  // check for system-events
  XMLElement * element = xmlDocument.FirstChildElement("sys");
  if (element != NULL) {
    const char * rr_cmd = "-unknown-";
    if (element->QueryStringAttribute("cmd", &rr_cmd) == XML_SUCCESS) {
      if ((strcmp(rr_cmd, "stop")==0) || (strcmp(rr_cmd, "ebreak")==0) || (strcmp(rr_cmd, "shutdown")==0)){
        Serial.println("EMERGENCY BREAK - stopping all trains! " + String(rr_cmd));
        targetTrainSpeed = 0;
        setTrainSpeed(0);
        delay(1000);
        return;
      }
    }
  }

  //XMLElement * element = xmlDocument.FirstChildElement("lc");
  element = xmlDocument.FirstChildElement("lc");
  if (element == NULL) {
    Serial.println("<lc> node not found. Message disregarded.");
    return;
  }

  // query addr attribute. This is the MattzoController id.
  // If this does not equal the controllerID of this controller, the message is disregarded.
  int rr_addr = 0;
  if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
    Serial.println("addr attribute not found or wrong type. Message disregarded.");
    return;
  }
  Serial.println("addr: " + String(rr_addr));
  if (rr_addr != controllerID) {
    Serial.println("Message disgarded, as it is not for me (" + String(controllerID) + ")");
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

  // query V attribute. This is speed information for the loco and ranges from 0 to 1023.
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
}


void reconnectHUB(){
  // init hubs from list and send "disconnected" information if lost
  for (int i = 0; i < NUM_HUBS; i++){
    if (!myHubs[i].isConnected() && !myHubs[i].isConnecting()){

      // send "connection lost" message
      if (String(myHubData[i][HUB_STATUS]) == String("true")) {
        sendMQTTMessage("roc2bricks/connectionStatus", String(myHubData[i][HUB_ADDRESS]) + " disconnected");
        myHubData[i][HUB_STATUS] = "false";
      }

      myHubs[i].init(myHubData[i][HUB_ADDRESS]);
    }
  }

  // connect to the hubs from the list and send "connected" information
  for (int i = 0; i < NUM_HUBS; i++){
    if (myHubs[i].isConnecting() && (myHubs[i].getHubType() == POWERED_UP_HUB)) {
      
      myHubs[i].connectHub();

      if (myHubs[i].isConnected()) {

      // send "connected" message
        sendMQTTMessage("roc2bricks/connectionStatus", String(myHubData[i][HUB_ADDRESS]) + " connected");
        myHubData[i][HUB_STATUS] = "true";

      } else {
        myHubData[i][HUB_STATUS] = "false";
      }
    }
  }
}


// set powered up motor speed
void setTrainSpeed(int newTrainSpeed){
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
      switch (String(myHubData[i][HUB_MOTORPORT]).indexOf("A")) {
         case 0: // "A"
            myHubs[i].setMotorSpeed(hubPortA, power);
            delay(DELAY);
            break;
         case 1: // "-A"
            myHubs[i].setMotorSpeed(hubPortA, -power);
            delay(DELAY);
            break;
         default:
            break;
      }

      switch (String(myHubData[i][HUB_MOTORPORT]).indexOf("B")) {
         case 0: // "B"
            myHubs[i].setMotorSpeed(hubPortB, power);
            delay(DELAY);
            break;
         case 1: // "-B"
            myHubs[i].setMotorSpeed(hubPortB, -power);
            delay(DELAY);
            break;
         default:
            break;
      }

      // Set integrated powered up hub light
      myHubs[i].setLedColor(ledColor);
    }
  }

  Serial.println("Motor speed set to: " + String(newTrainSpeed));
}


// gently adapt train speed (increase/decrease slowly)
void accelerateTrainSpeed() {
  if (currentTrainSpeed != targetTrainSpeed) {
    if (targetTrainSpeed == 0){
      // stop -> execute immediately
      setTrainSpeed(0);
    } else if (currentMillis - lastAccelerate >= ACCELERATION_INTERVAL) {
      lastAccelerate = millis();

      int nextSpeed;
      // accelerate / brake gently
      if (currentTrainSpeed < targetTrainSpeed) {
        nextSpeed = min(currentTrainSpeed + ACCELERATE_STEP, targetTrainSpeed);
      } else {
        nextSpeed = max(currentTrainSpeed - BRAKE_STEP, targetTrainSpeed);
      }
      setTrainSpeed(nextSpeed);
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

  currentMillis = millis();
  accelerateTrainSpeed();

  sendMQTTPing();
  sendMQTTBatteryLevel();
}
