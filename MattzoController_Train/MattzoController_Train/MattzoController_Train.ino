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
#include "PoweredUpHub.h"  // https://github.com/corneliusmunz/legoino

using namespace tinyxml2;

/* Mattzo Controller spezific */
String eepromIDString = "MattzoTrainController";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 22;              // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerID;                        // controller id. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.


/* WLAN settings */
const char* WIFI_SSID = "MWE2";                   // SSID of your WLAN
const char* WIFI_PASSWORD = "Oomph$4Oomph$4";     // password of your WLAN


/* MQTT settings */
const char* MQTT_BROKER_IP = "192.168.1.106";            // IP address of the server on which the MQTT is installed
const int MQTT_BROKER_PORT = 1883;                       // Port of the MQTT broker
String mqttClientName;                                   // Name of the MQTT client (me) with which messages are sent
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.


/* Timing */

/* Current time for event timing */
unsigned long currentMillis = millis();



/* Send a ping to announce that you are still alive */
const int SEND_PING_INTERVAL = 10000; // interval for sending pings in milliseconds
unsigned long lastPing = millis();    // time of the last sent ping

/* Send the battery level  */
const int SEND_BATTERYLEVEL_INTERVAL = 60000; // interval for sending battery level in milliseconds
unsigned long lastBatteryLevel = millis();    // time of the last sent battery level



/* LEGO Powered up */

PoweredUpHub myHubs[2];  // all the hubs (two in this case :)

// the four different Values of the second dimension of the hub-Array (the first dimension is the hub itselfe
const int HUB_NAME         = 0; // name of the hub
const int HUB_ADDRESS      = 1; // MAC address
const int HUB_STATUS       = 2; // connectionstatus of the hub ("true" = connected, "false" = not connected)
const int HUB_MOTORPORT    = 3; // ports with motors (A, -A,  AB, -AB, -A-B, A-B, B, -B), a minus indicates an inverse rotation direction
//const int HUB_SPEEDFACTOR  = 4; // multiplier for the speed of a motor in relation to the first motor (1 = same speed, same direction, -1 = same speed, inverted direction, -2 = double speed, inverted direction)

// note: only hubs are allowed, remotes will be refused

//              |-- number of Hubs
//              |
//              |  |-- { "Name", "Address", "connectionStatus", "port and direction"}
//              |  |
//              v  v
char* myHubData[2][4]=
{{"Hub 1", "90:84:2b:17:c5:b3", "false", "A"} //, "1"}
,{"Hub 2", "00:81:f9:e9:d1:24", "false", "-A"} //, "-1"}
};

PoweredUpHub::Port hubPortA = PoweredUpHub::Port::A; // port A
PoweredUpHub::Port hubPortB = PoweredUpHub::Port::B; // port B


const int ACCELERATION_BREAK_INTERVAL = 1;   // pause between individual speed adjustments in milliseconds
unsigned long lastAccelerate = millis();     // time of the last speed adjustment
const int accelerateStep = 1;
int currentTrainSpeed = 0;                   // current speed of this train
int targetTrainSpeed = 0;                    // Target speed Speed of this train


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
  int maxControllerId = 65000;              // The controller id is a number between 0 and 65000

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
    controllerID = random(1, maxControllerId);
    controllerIDHiByte  = controllerID / 256;
    controllerIDLowByte = controllerID % 256;
    EEPROM.write(paramsStartingPosition, controllerIDHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerIDLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

  }
  Serial.println("controller ID " + String(controllerID));

}

void initWIFI() {
  Serial.println("MAC: " + WiFi.macAddress());
  //Serial.print("Connecting to SSID " + String(WIFI_SSID) + " ");
  //WiFi.setSleepMode(WIFI_NONE_SLEEP);
  reconnectWIFI();
}

void reconnectWIFI() {

  Serial.print("Connecting to SSID " + String(WIFI_SSID) + " ");
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
  Serial.print("Connected to SSID " + String(WIFI_SSID) + " ");
  Serial.println(WiFi.localIP());
}

void initMQTT() {
  // set MQTT client name
  mqttClientName = eepromIDString + " " + String(controllerID);
  mqttClientName.toCharArray(mqttClientName_char, mqttClientName.length() + 1);

  client.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
  client.setCallback(callbackMQTT);
  client.setBufferSize(2048);
}

void reconnectMQTT() {
  Serial.println("MQTT connecting ...");
  while (!client.connected()) {
      if (!client.connect(mqttClientName_char)) {
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
    client.publish("roc2bricks/ping", mqttClientName_char);
  }
}

void sendMQTTMessage(String messageType, String message){

  char messageType_char[50];
  messageType.toCharArray(messageType_char, message.length() + 1);
  
  char message_char[100];
  message.toCharArray(message_char, message.length() + 1);
  
  Serial.println("send " + messageType + ": " + message);
  client.publish(messageType_char, message_char);
}

void sendMQTTBatteryLevel(){
  if (currentMillis - lastBatteryLevel >= SEND_BATTERYLEVEL_INTERVAL) {
    lastBatteryLevel = millis();

    for (int i = 0; i < 2; i++){
      if (myHubs[i].isConnected()) {
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
      Serial.print((char)payload[i]);
      msg[i] = (char)payload[i];
  }
   Serial.println();

  msg[length] = '\0';
  Serial.println(msg);

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

      if ((strcmp(rr_cmd, "stop")==0) || (strcmp(rr_cmd, "ebreak")==0)){
        Serial.println("! NOTHALT ! " + String(rr_cmd));
        SetTrainSpeed(0);
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

  // query addr1 attribute. This is the MattzoController id.
  // If this does not equal the controllerID of this controller, the message is disregarded.
  int rr_oid = 0;
  if (element->QueryIntAttribute("oid", &rr_oid) != XML_SUCCESS) {
    Serial.println("oid attribute not found or wrong type. Message disregarded.");
    return;
  }
  Serial.println("oid: " + String(rr_oid));
  if (rr_oid != controllerID) {
    Serial.println("Message disgarded, as it is not for me (" + String(controllerID) + ")");
    return;
  }

  // query port1 attribute. This is port id of the port to which the switch is connected.
  // If the controller does not have such a port, the message is disregarded.
  int rr_speed = 0;
  if (element->QueryIntAttribute("V", &rr_speed) != XML_SUCCESS) {
    Serial.println("V attribute not found or wrong type. Message disregarded.");
    return;
  }
  Serial.println("speed: " + String(rr_speed));
  if ((rr_speed < 0) || (rr_speed > 100)) {
    Serial.println("Message disgarded, as this controller does not have such a port.");
    return;
  }
  SetTrainSpeed(rr_speed);
  //targetTrainSpeed = rr_speed;

}


void reconnectHUB(){

  // init hubs from list and send "connection lost" information if lost
  for (int i = 0; i < 2; i++){
    if (!myHubs[i].isConnected() && !myHubs[i].isConnecting()){

      // send "connection lost" message
      if (String(myHubData[i][HUB_STATUS]) == String("true")) {
        sendMQTTMessage("roc2bricks/connectionStatus", String(myHubData[i][HUB_ADDRESS]) + " connection lost");
        myHubData[i][HUB_STATUS] = "false";
      }

      myHubs[i].init(myHubData[i][HUB_ADDRESS]);
    }
  }

  // connect to the hubs from the list and send "connected" information
  for (int i = 0; i < 2; i++){
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


// set train speed
void SetTrainSpeed(int newTrainSpeed){
  Serial.println("set speed: " + String(newTrainSpeed));

  for (int i = 0; i < 2; i++){

    if (myHubs[i].isConnected()) {

      if (currentTrainSpeed != newTrainSpeed){
        currentTrainSpeed = newTrainSpeed;
      }
      
      switch (String(myHubData[i][HUB_MOTORPORT]).indexOf("A")) {
         case 0: // "A"
            myHubs[i].setMotorSpeed(hubPortA, currentTrainSpeed);
            break;
         case 1: // "-A"
            myHubs[i].setMotorSpeed(hubPortA, -currentTrainSpeed);
            break;
         default:
            break;
      }

      String bValue = String(myHubData[i][HUB_MOTORPORT]);
      bValue = bValue.substring(bValue.indexOf("A") + 1);

      switch (String(myHubData[i][HUB_MOTORPORT]).indexOf("B")) {
         case 0: // "B"
            myHubs[i].setMotorSpeed(hubPortB, currentTrainSpeed);
            break;
         case 1: // "-B"
            myHubs[i].setMotorSpeed(hubPortB, -currentTrainSpeed);
            break;
         default:
            break;
      }


    }

  }
}


// set train speed (increase/decrease slowly)
void accelerateTrainSpeed(int trainSpeed){

    if (currentMillis - lastAccelerate >= ACCELERATION_BREAK_INTERVAL) {
      lastAccelerate = millis();

      if (currentTrainSpeed > trainSpeed){
        SetTrainSpeed(currentTrainSpeed - accelerateStep);
      } else if (currentTrainSpeed < trainSpeed) {
        SetTrainSpeed(currentTrainSpeed + accelerateStep);
      }

    }
  
}


// testing, testing, testing ......
void doTrainMotorThings(){
//lastHubThing
    //if (currentMillis - lastHubThing >= TICKS_BETWEEN_HUBTHINGS) {


    //myTrain.getLedColor

    //char hubName[] = "myTrainHub";
    //trainHub1.setLedColor(GREEN);
    //delay(500);
    //trainHub1.setLedColor(RED);
    //delay(500);
    //trainHub1.setMotorSpeed(_portA, 35);
    //delay(1000);
    //trainHub1.stopMotor(_portA);
    //delay(1000);
    //trainHub1.setMotorSpeed(_portA, -35);
    //delay(1000);
    //trainHub1.stopMotor(_portA);
    //delay(1000);

/*
    if (currentMillis - lastBatteryLevel >= SEND_BATTERYLEVEL_INTERVAL) {
      Serial.println("BatteryLevel: " + String(myTrainHub.getBatteryLevel()));
      //Serial.println("current speed: " + String(currentTrainSpeed));
      lastBatteryLevel = millis();
    }
*/
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

  //if (trainHub1.isConnected()) {
  //  doTrainMotorThings();
  //  //accelerateTrainSpeed(targetTrainSpeed);
  //}

  // Send ping?
  //sendMQTTPing();
  sendMQTTBatteryLevel();

}
