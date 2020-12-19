// MattzoSensorController Firmware
// Author: Dr. Matthias Runte
// Libraries can be downloaded easily from within the Arduino IDE using the library manager.

// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <EEPROM.h>  // EEPROM library
#include <ESP8266WiFi.h>  // WiFi library
#include <PubSubClient.h>  // MQTT library
#include "MattzoSensorController_Configuration.h"  // this file should be placed in the same folder
#include "MattzoController_Library.h"  // this file needs to be placed in the Arduino library folder

String eepromIDString = "MattzoSensorController";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 22;  // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerNo;  // controllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 65000;

String mqttClientName;
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

const int NUM_SENSORS = 4; // Number of connectable sensors
uint8_t SENSOR_PIN[NUM_SENSORS];  // Digital PINs for input of hall, reed or other digital signals
uint8_t LED_PIN[NUM_SENSORS];  // Digital PINs for input of hall, reed or other digital signals

const int SENSOR_RELEASE_TICKS = 500;  // time in milliseconds until sensor is reported to be released after it actually has lost contact
bool sensorState[NUM_SENSORS];
int lastSensorContactMillis[NUM_SENSORS];

WiFiClient espClient;
PubSubClient client(espClient);



void setup() {
  Serial.begin(115200);
  randomSeed(ESP.getCycleCount());
  Serial.println("");
  Serial.println("MattzoController booting...");

  SENSOR_PIN[0] = D1;
  SENSOR_PIN[1] = D2;
  SENSOR_PIN[2] = D5;
  SENSOR_PIN[3] = D6;
  LED_PIN[0] = D3;
  LED_PIN[1] = D4;
  LED_PIN[2] = D7;
  LED_PIN[3] = D8;

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PIN[i], INPUT);
    pinMode(LED_PIN[i], OUTPUT);
    sensorState[i] = false;
  }

  // all LEDs off
  allBlink(false, 0);

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
    allBlink(true, 0);
    delay(400);
    allBlink(false, 0);
    delay(400);
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

void allBlink(boolean onOff, int blinkType) {
  if (blinkType == 0) {
    setLED(0, onOff);
    setLED(1, onOff);
    setLED(2, onOff);
    setLED(3, onOff);
  } else if (blinkType == 1) {
    setLED(0, onOff);
    setLED(1, !onOff);
    setLED(2, onOff);
    setLED(3, !onOff);
  } else if (blinkType == 2) {
    // Knight Rider
    const int KNIGHT_RIDER_DELAY = 100;
    setLED(0, true);
    delay(KNIGHT_RIDER_DELAY);
    setLED(0, false);
    setLED(1, true);
    delay(KNIGHT_RIDER_DELAY);
    setLED(1, false);
    setLED(2, true);
    delay(KNIGHT_RIDER_DELAY);
    setLED(2, false);
    setLED(3, true);
    delay(KNIGHT_RIDER_DELAY);
    setLED(3, false);
    setLED(2, true);
    delay(KNIGHT_RIDER_DELAY);
    setLED(2, false);
    setLED(1, true);
    delay(KNIGHT_RIDER_DELAY);
    setLED(1, false);
    setLED(0, true);
    delay(KNIGHT_RIDER_DELAY);
    setLED(0, false);
  }
  
  return;
  
  if (onOff) {
    for (int i=0; i < NUM_SENSORS; i++) {
      if (blinkType == 0) {
        // all flash
        setLED(i, onOff);
      } else {
        // flash alternatively
        setLED(i, onOff ^ (i % 2 == 0));
      }
    }
  }
}

void setLED(int index, bool ledState) {
  if (ledState) {
    digitalWrite(LED_PIN[index], LOW);
  } else {
    digitalWrite(LED_PIN[index], HIGH);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
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
}

void reconnectMQTT() {
  while (!client.connected()) {
    allBlink(true, 1);
    Serial.println("Reconnecting MQTT...");

    String lastWillMessage = String(mqttClientName_char) + " " + "last will and testament";
    char lastWillMessage_char[lastWillMessage.length() + 1];
    lastWillMessage.toCharArray(lastWillMessage_char, lastWillMessage.length() + 1);

    if (!client.connect(mqttClientName_char, "roc2bricks/lastWill", 0, false, lastWillMessage_char)) {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      for (int i=0; i < 5; i++) {
        allBlink(false, 1);
        delay(500);
        allBlink(true, 1);
        delay(500);
      }
    }
  }
  allBlink(false, 2);
  Serial.println("MQTT Connected.");
}


void loop() {
  int sensorValue;
  
  if (!client.connected()) {
      reconnectMQTT();
  }
  client.loop();

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValue = digitalRead(SENSOR_PIN[i]);

/*
    if (ticksBetweenPingsCounter % 100 == 0) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(sensorValue);
   } 
*/
    if (sensorValue == LOW) {
      // Contact -> report contact immediately
      if (!sensorState[i]) {
        Serial.println("Sensor " + String(i) + ": Contact!");
        sendMQTTSensorEvent(i, true);
        sensorState[i] = true;
        setLED(i, true);
      }
      lastSensorContactMillis[i] = millis();
    } else { 
      // No contact for SENSOR_RELEASE_TICKS milliseconds -> report sensor has lost contact
      if (sensorState[i] && (millis() > lastSensorContactMillis[i] + SENSOR_RELEASE_TICKS)) {
        Serial.println("Sensor " + String(i) + ": Released!");
        sendMQTTSensorEvent(i, false);
        sensorState[i] = false;
        setLED(i, false);
      }
    }
  }

  sendMQTTPing(&client, mqttClientName_char);
}

void sendMQTTSensorEvent(int sensorPort, int sensorState) {
  String sensorRocId = eepromIDString + String(controllerNo) + "-" + String(sensorPort + 1);  // e.g. "MattzoController12345-3"
  String stateString;
  if (sensorState)
    stateString = "true";
  else
    stateString = "false";

  // compile mqtt message. Parameters:
  //   id: Combination of sensor name and port (e.g. MattzoController12345-3). The reported port (the "logic" port) is 1 count higher than the internal port number in the sensor, e.g. port 2 in the sensor equals 3 in Rocrail)
  //   bus: controller number
  //   address: port number (internal port number plus 1)
  // both id or bus/address can be used in Rocrail. If id is used, it superseeds the combination of bus and address
  String mqttMessage = "<fb id=\"" + sensorRocId + "\" bus=\"" + String(controllerNo) + "\" addr=\"" + String(sensorPort + 1) + "\" state=\"" + stateString + "\"/>";
  Serial.println("Sending MQTT message: " + mqttMessage);
  char mqttMessage_char[255];   // message is usually 61 chars, so 255 chars should be enough
  mqttMessage.toCharArray(mqttMessage_char, mqttMessage.length() + 1);
  client.publish("rocrail/service/client", mqttMessage_char);
}
