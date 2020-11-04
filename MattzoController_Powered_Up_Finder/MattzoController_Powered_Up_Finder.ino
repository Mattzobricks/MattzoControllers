// MattzoControllerPoweredUpFinder Firmware
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
#include <PoweredUpRemote.h>
#include <MattzoController_Network_Configuration.h>  // this file needs to be placed in the Arduino library folder

using namespace tinyxml2;

/* Mattzo Controller spezific */
String eepromIDString = "MattzoControllerPUFinder";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 24;                 // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerID;                           // controller id. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.

/* MQTT */
String mqttClientName;                                   // Name of the MQTT client (me) with which messages are sent
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

int i = 0;
PoweredUpHub myHub[100];

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

  client.setServer(MQTT_BROKER_IP, 1883);
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

void sendMQTTMessage(String messageType, String message){

  char messageType_char[50];
  messageType.toCharArray(messageType_char, message.length() + 1);
  
  char message_char[100];
  message.toCharArray(message_char, message.length() + 1);
  
  Serial.println("send " + messageType + ": " + message);
  client.publish(messageType_char, message_char);
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

}





void loop() {
  if (WiFi.status() != WL_CONNECTED){
    reconnectWIFI();
  }

  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  if (!myHub[i].isConnected() && !myHub[i].isConnecting()) {
    myHub[i].init(); // initalize the PoweredUpHub instance
  }

  // connect flow. Search for BLE services and try to connect if the uuid of the hub is found
  if (myHub[i].isConnecting()) {
    myHub[i].connectHub();
    if (myHub[i].isConnected()) {

      String hubType;

      switch(myHub[i].getHubType()){
        case 2: hubType = "BOOST_MOVE_HUB";
                break;
        case 3: hubType = "POWERED_UP_HUB";
                break;
        case 4: hubType = "POWERED_UP_REMOTE";
                break;
        case 5: hubType = "DUPLO_TRAIN_HUB";
                break;
        case 6: hubType = "CONTROL_PLUS_HUB";
                break;
        default:hubType = "UNKNOWNHUB";
                break;
        }
  
      String hubAddress = myHub[i].getHubAddress().toString().c_str();
      
      Serial.println("Connected to HUB[" + String(i+1) + "]: " + hubType + ": " + hubAddress);
      sendMQTTMessage("roc2bricks/connectionStatus", "discovered " + hubType + ": " + hubAddress);

      i = i + 1;
      myHub[i].init();
    } else {
      Serial.println("Failed to connect to HUB");
    }
  }
}
