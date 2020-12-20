// MattzoSignalController Firmware
// Author: Dr. Matthias Runte
// Libraries can be downloaded easily from within the Arduino IDE using the library manager.
// TinyXML2 must be downloaded from https://github.com/leethomason/tinyxml2 (required files: tinyxml2.cpp, tinyxml2.h)

// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <EEPROM.h>  // EEPROM library
#include <ESP8266WiFi.h>  // WiFi library
#include <PubSubClient.h>  // MQTT library
#include <tinyxml2.h>  // tiny xml 2 library
#include "MattzoSignalController_Configuration.h"  // this file should be placed in the same folder
#include "MattzoController_Library.h"  // this file needs to be placed in the Arduino library folder

using namespace tinyxml2;

String eepromIDString = "MattzoSignalController";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 22;  // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerNo;  // controllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 65000;

String mqttClientName;
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

WiFiClient espClient;
PubSubClient client(espClient);




void setup() {
  Serial.begin(115200);
  randomSeed(ESP.getCycleCount());
  Serial.println("");
  Serial.println("MattzoController booting...");

  // initialize pins
  for (int i = 0; i < NUM_SIGNALPORTS; i++) {
    pinMode(SIGNALPORT_PIN[i], OUTPUT);
  }
  pinMode(STATUS_LED_PIN, OUTPUT);
  setStatusLED(true);  // LED on

  loadPreferences();
  setupWifi();
  setupSysLog(mqttClientName_char);
  setupMQTT();

  mcLog("MattzoController setup completed.");
}

void loadPreferences() {
  int i;
  int controllerNoHiByte;
  int controllerNoLowByte;

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
    // load controller number from preferences
    controllerNoHiByte = EEPROM.read(paramsStartingPosition);
    controllerNoLowByte = EEPROM.read(paramsStartingPosition + 1);
    controllerNo = controllerNoHiByte * 256 + controllerNoLowByte;
    Serial.println("Loaded controllerNo from EEPROM: " + String(controllerNo));

  } else {
    // preferences not initialized yet -> initialize controller
    // this runs only a single time when starting the controller for the first time

    // Wait a bit to give the user some time to open the serial console...
    delay (5000);
    
    Serial.println("Initializing controller preferences on first start-up...");
    for (i = 0; i < eepromIDString.length(); i++) {
      EEPROM.write(i, eepromIDString.charAt(i));
    }

    // assign random controller number between 1 and 65000 and store in EEPROM
    controllerNo = random(1, MAX_CONTROLLER_ID);
    controllerNoHiByte = controllerNo / 256;
    controllerNoLowByte = controllerNo % 256;
    EEPROM.write(paramsStartingPosition, controllerNoHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerNoLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

    Serial.println("Assigned random controller no " + String(controllerNo) + " and stored to EEPROM");
  }

  // set MQTT client name
  mqttClientName = eepromIDString + String(controllerNo);
  mqttClientName.toCharArray(mqttClientName_char, mqttClientName.length() + 1);
}

void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  while (WiFi.status() != WL_CONNECTED) {
    setStatusLED(true);
    delay(400);
    setStatusLED(false);
    delay(400);
    Serial.print(".");
  }
 
  Serial.println("");
  mcLog("WiFi connected. My IP address is " + WiFi.localIP().toString() + ".");
}
 
void setupMQTT () {
  client.setServer(MQTT_BROKER_IP, 1883);
  client.setCallback(mqttCallback);
  client.setBufferSize(2048);
  client.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);   // keep alive interval
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length+1];
  for (int i = 0; i < length; i++) {
      msg[i] = (char)payload[i];
  }
  msg[length] = '\0';

  mcLog("Received MQTT message [" + String(topic) + "]: " + String(msg));

  XMLDocument xmlDocument;
  if(xmlDocument.Parse(msg)!= XML_SUCCESS) {
    mcLog("Error parsing.");
    return;
  }

  mcLog("Parsing XML successful.");
  XMLElement * element = xmlDocument.FirstChildElement("co");
  if (element == NULL) {
    mcLog("<co> node not found. Message disregarded.");
    return;
  }

  mcLog("<co> node found.");

  // query addr attribute. This is the MattzoController id.
  // If this does not equal the ControllerNo of this controller, the message is disregarded.
  int rr_addr = 0;
  if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
    mcLog("addr attribute not found or wrong type. Message disregarded.");
    return;
  }
  mcLog("addr: " + String(rr_addr));
  if (rr_addr != controllerNo) {
    mcLog("Message disgarded, as it is not for me (" + String(controllerNo) + ")");
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
  const char * rr_cmd = "-unknown-";
  if (element->QueryStringAttribute("cmd", &rr_cmd) != XML_SUCCESS) {
    mcLog("cmd attribute not found or wrong type.");
    return;
  }
  mcLog("cmd: " + String(rr_cmd));

  // set signal LED for the port on/off
  if (strcmp(rr_cmd, "on")==0) {
    setSignalLED(rr_port - 1, true);
  } else if (strcmp(rr_cmd, "off")==0) {
    setSignalLED(rr_port - 1, false);
  } else {
    mcLog("Signal port command unknown - message disregarded.");
    return;
  }
}

void setSignalLED(int index, bool ledState) {
  digitalWrite(SIGNALPORT_PIN[index], ledState ? LOW : HIGH);
}

void setStatusLED(bool ledState) {
  digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
}

void reconnectMQTT() {
  while (!client.connected()) {
    mcLog("Reconnecting MQTT...");

    String lastWillMessage = String(mqttClientName_char) + " " + "last will and testament";
    char lastWillMessage_char[lastWillMessage.length() + 1];
    lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

    if (!client.connect(mqttClientName_char, "roc2bricks/lastWill", 0, false, lastWillMessage_char)) {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      for (int i = 0; i < 5; i++) {
        setStatusLED(false);
        delay(500);
        setStatusLED(true);
        delay(500);
      }
    }
  }
  setStatusLED(false);
  client.subscribe("rocrail/service/command");
  mcLog("MQTT connected, listening on topic [rocrail/service/command].");
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  sendMQTTPing(&client, mqttClientName_char);
}
