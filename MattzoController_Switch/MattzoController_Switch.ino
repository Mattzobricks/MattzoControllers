// MattzoSwitchController Firmware
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define MATTZO_CONTROLLER_TYPE "MattzoSwitchController"
#include <ESP8266WiFi.h>                          // WiFi library for ESP-8266
#include <Servo.h>                                // Servo library
#include "MattzoSwitchController_Configuration.h" // this file should be placed in the same folder
#include "MattzoController_Library.h"             // this file needs to be placed in the Arduino library folder

// Create servo objects to control servos
Servo servo[NUM_SWITCHPORTS];

// Default values for TrixBrix switches (in case servo angles are not transmitted)
const int SERVO_MIN = 70;
const int SERVO_START = 78;  // position after boot-up
const int SERVO_MAX = 85;

// Delay between to switch operations
const int SWITCH_DELAY = 200;



void setup() {
    // initialize pins and turn servos to start position
    for (int i = 0; i < NUM_SWITCHPORTS; i++) {
      servo[i].attach(SWITCHPORT_PIN[i]);
      servo[i].write(SERVO_START);
      delay(SWITCH_DELAY);
    }

    // load config from EEPROM, initialize Wifi, MQTT etc.
    setupMattzoController();
}

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
  XMLElement * element = xmlDocument.FirstChildElement("sw");
  if (element == NULL) {
    mcLog("<sw> node not found. Message disregarded.");
    return;
  }

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
  if ((rr_port1 < 1 || rr_port1 > NUM_SWITCHPORTS) && (rr_port1 < 1001 || rr_port1 > 1004)) {
    mcLog("Message disgarded, this controller does not have such a port.");
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
    setServoAngle(servoPort1 - 1, servoAngle);
    setServoAngle(servoPort2 - 1, servoAngle);
  }
  else {
    mcLog("Turning servo on port " + String(rr_port1) + " to angle " + String(servoAngle));
    setServoAngle(rr_port1 - 1, servoAngle);
  }
}

void setServoAngle(int servoIndex, int servoAngle) {
  if (servoIndex >= 0 && servoIndex < NUM_SWITCHPORTS) {
    servo[servoIndex].write(servoAngle);
    delay(SWITCH_DELAY);
  }
  else {
    // this should not happen
    mcLog("WARNING: servo index " + String(servoIndex) + " out of range!");
  }
}

void loop() {
  loopMattzoController();
}
