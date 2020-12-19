// MattzoSwitchController Firmware
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
#include <Servo.h>  // servo library
#include <tinyxml2.h>  // tiny xml 2 library
#include "MattzoSwitchController_Configuration.h"  // this file should be placed in the same folder
#include "MattzoController_Library.h"  // this file needs to be placed in the Arduino library folder

using namespace tinyxml2;

String eepromIDString = "MattzoSwitchController";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 22;  // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerNo;  // controllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 65000;

String mqttClientName;
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

const int NUM_SWITCHPORTS = 8; // Number of switch ports
uint8_t SWITCHPORT_PIN[NUM_SWITCHPORTS];  // Digital PINs for output

WiFiClient espClient;
PubSubClient client(espClient);

// create servo objects to control servos
Servo servo[NUM_SWITCHPORTS];

// Default values for TrixBrix switches (in case servo angles are not transmitted)
const int SERVO_MIN = 70;
const int SERVO_START = 78;  // position after boot-up
const int SERVO_MAX = 85;

// delay between to switch operations
const int SWITCH_DELAY = 200;



void setup() {
    Serial.begin(115200);
    randomSeed(ESP.getCycleCount());
    Serial.println("");
    Serial.println("MattzoController booting...");

    servo[0].attach(D0);
    servo[1].attach(D1);
    servo[2].attach(D2);
    servo[3].attach(D3);
    servo[4].attach(D4);
    servo[5].attach(D5);
    servo[6].attach(D6);
    servo[7].attach(D7);

    for (int i = 0; i < NUM_SWITCHPORTS; i++) {
      servo[i].write(SERVO_START);
      delay(SWITCH_DELAY);
    }

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
      delay(1000);
      Serial.print(".");

      // TODO: Support WPS! Store found Wifi network found via WPS in EEPROM and use next time!
    }
 
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
 
// Setup MQTT
void setupMQTT() {
  client.setServer(MQTT_BROKER_IP, 1883);
  client.setCallback(mqttCallback);
  client.setBufferSize(2048);
  client.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);   // keep alive interval
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length+1];
  for (int i = 0; i < length; i++) {
      // Serial.print((char)payload[i]);
      msg[i] = (char)payload[i];
  }

  msg[length] = '\0';
  mcLog(msg);

  XMLDocument xmlDocument;
  if(xmlDocument.Parse(msg)!= XML_SUCCESS){
    mcLog("Error parsing");
    return;
  }

  mcLog("Parsing XML successful");
  XMLElement * element = xmlDocument.FirstChildElement("sw");
  if (element == NULL) {
    mcLog("<sw> node not found. Message disregarded.");
    return;
  }

  mcLog("<sw> node found.");

  // query addr1 attribute. This is the MattzoController id.
  // If this does not equal the ControllerNo of this controller, the message is disregarded.
  int rr_addr1 = 0;
  if (element->QueryIntAttribute("addr1", &rr_addr1) != XML_SUCCESS) {
    mcLog("addr1 attribute not found or wrong type. Message disregarded.");
    return;
  }
  mcLog("addr1: " + String(rr_addr1));
  if (rr_addr1 != controllerNo) {
    mcLog("Message disgarded, as it is not for me (" + String(controllerNo) + ")");
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
  if ((rr_port1 < 1 || rr_port1 > NUM_SWITCHPORTS) && (rr_port1 < 1001 || rr_port1 > 1004)) {
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

  // query param1 attribute. This is the "straight" position of the switch servo motor.
  // defaults to SERVO_MIN
  int rr_param1 = SERVO_MIN;
  if (element->QueryIntAttribute("param1", &rr_param1) != XML_SUCCESS) {
    mcLog("param1 attribute not found or wrong type. Using default value.");
  }
  mcLog("param1: " + String(rr_param1));

  // query value1 attribute. This is the "turnout" position of the switch servo motor.
  // defaults to SERVO_MAX
  int rr_value1 = SERVO_MAX;
  if (element->QueryIntAttribute("value1", &rr_value1) != XML_SUCCESS) {
    mcLog("value1 attribute not found or wrong type. Using default value.");
  }
  mcLog("value1: " + String(rr_value1));

  // check command string and prepare servo angle
  // servo angle will only be used to flip a standard or one side of a triple switch - not for double slip switches!
  int switchCommand;
  int servoAngle;
  if (strcmp(rr_cmd, "straight")==0) {
    switchCommand = 1;
    servoAngle = rr_param1;
  } else if (strcmp(rr_cmd, "turnout")==0) {
    switchCommand = 0;
    servoAngle = rr_value1;
  } else {
    mcLog("Switch command unknown - message disregarded.");
    return;
  }

  // Check if port is between 1001 and 1004 -> special rules for trixbrix double slip switches apply!
  if (rr_port1 >= 1001 && rr_port1 <= 1004) {
    int servoPort1;
    int servoPort2;

    // port 1001 -> double slip switch 1 / side A / ports 1 and 2
    if (rr_port1 == 1001) {
      servoPort1 = 1;
      servoPort2 = 2;
    // port 1002 -> double slip switch 1 / side B / ports 3 and 4
    } else if (rr_port1 == 1002) {
      servoPort1 = 3;
      servoPort2 = 4;
    // port 1003 -> double slip switch 2 / side A / ports 5 and 6
    } else if (rr_port1 == 1003) {
      servoPort1 = 5;
      servoPort2 = 6;
    // port 1004 -> double slip switch 2 / side B / ports 7 and 8
    } else if (rr_port1 == 1004) {
      servoPort1 = 7;
      servoPort2 = 8;
    }

    mcLog("Turning double slip switch servos on port " + String(servoPort1) + " and " + String(servoPort2) + " to angle " + String(servoAngle));
    servo[servoPort1-1].write(servoAngle);
    delay(SWITCH_DELAY);

    servo[servoPort2-1].write(servoAngle);
    delay(SWITCH_DELAY);

  } else {
    mcLog("Turning servo on port " + String(rr_port1) + " to angle " + String(servoAngle));

    servo[rr_port1 - 1].write(servoAngle);
    delay(SWITCH_DELAY);
  }
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
        mcLog(". Retrying in 5 seconds...");
        delay(5000);
      }
  }
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
