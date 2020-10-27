// MattzoSwitchController Firmware V0.1
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
#include <MattzoController_Network_Configuration.h>

using namespace tinyxml2;

String eepromIDString = "MattzoTrainController4PF";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 24;  // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerNo;  // controllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 16383;

String mqttClientName;
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

const int NUM_FUNCTIONS = 2;  // if increased, the fn1, fn2... defintions must be enhanced as well. Also check for usage of those parameters and extend code accordingly!
#define fn1 D0  // Output PIN for Rocrail Function 1 (e.g. train headlights)
#define fn2 D8  // Output PIN for Rocrail Function 2 (e.g. train taillights, reverse headlights, interior lighting etc.)

const int MOTORSHIELD_TYPE = 2; // motor shield type. 1 = L298N, 2 = L9110
#define enA D1  // PWM signal for motor A. Relevant for L298N only.
#define in1 D2  // motor A direction control (forward)
#define in2 D3  // motor A direction control (reverse)
#define enB D5  // PWM signal for motor B. Relevant for L298N only.
#define in3 D6  // motor B direction control (forward)
#define in4 D7  // motor B direction control (reverse)

const boolean REVERSE_A = false;  // if set to true, motor A is reversed, i.e. forward is backward and vice versa.
const boolean REVERSE_B = true;  // if set to true, motor B is reversed

/* Send the battery level  */
const int SEND_BATTERYLEVEL_INTERVAL = 60000; // interval for sending battery level in milliseconds
unsigned long lastBatteryLevelMsg = millis();    // time of the last sent battery level
const int BATTERY_PIN = A0;
const int VOLTAGE_MULTIPLIER = 20000/5000 - 1;   // Rbottom = 5 kOhm; Rtop = 20 kOhm; => voltage split factor
const int MAX_AI_VOLTAGE = 5100;  // maximum analog input voltage on pin A0. Usually 5000 = 5V = 5000mV. Can be slightly adapted to correct small deviations

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
    Serial.println("");
    Serial.println("MattzoController booting...");

    // initialize function pins
    pinMode(fn1, OUTPUT);
    pinMode(fn2, OUTPUT);

    // initialize motor pins
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // stop motors
    setTrainSpeed(0);

    loadPreferences();
    setup_wifi();
    setup_mqtt();
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

void setup_wifi() {
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
 
void setup_mqtt() {
  client.setServer(MQTT_BROKER_IP, 1883);
  client.setCallback(callback);
  client.setBufferSize(2048);
  client.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);   // keep alive interval
}

void sendMQTTBatteryLevel(){
  if (millis() - lastBatteryLevelMsg >= SEND_BATTERYLEVEL_INTERVAL) {
    lastBatteryLevelMsg = millis();
    int a0Value = analogRead(BATTERY_PIN);
    int voltage = map(a0Value, 0, 1023, 0, MAX_AI_VOLTAGE * VOLTAGE_MULTIPLIER);  // Battery Voltage in mV (Millivolt)
    if (voltage > 99999) voltage = 99999;

    Serial.println("sending battery level raw=" + String(a0Value) + ", mv=" + String(voltage));
    String batteryMessage = mqttClientName + " raw=" + String(a0Value) + ", mv=" + String(voltage);
    char batteryMessage_char[batteryMessage.length() + 1];  // client name + 5 digits for voltage in mV plus terminating char
    batteryMessage.toCharArray(batteryMessage_char, batteryMessage.length() + 1);

    client.publish("roc2bricks/battery", batteryMessage_char);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length+1];
  for (int i = 0; i < length; i++) {
      msg[i] = (char)payload[i];
  }
  msg[length] = '\0';
  Serial.println(msg);

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
    // Nevertheless, the id has no effect on the controller behaviour. Only the "addr" attribute is relevant for checking if the message is for this controller - see below.
    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      Serial.println("id attribute not found or wrong type.");
      return;
    }
    Serial.println("loco id: " + String(rr_id));
  
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
  } else {
    // check for fn message
    element = xmlDocument.FirstChildElement("fn");
    if (element == NULL) {
      Serial.println("<fn> node not found. Disregarding message...");
      return;
    }

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

    int functionPin;
    switch (rr_functionNo) {
      case 1:
        functionPin = fn1;
        break;
      case 2:
        functionPin = fn2;
        break;
      default:
        Serial.println("fnchanged out of range. Message disregarded.");
        return;
    }
    Serial.println("Function PIN: " + functionPin);

    // query fnchangedstate attribute. This is value if the function shall be set on or off
    const char * rr_state = "xxxxxx";  // expected values are "true" or "false"
    boolean functionState;
    if (element->QueryStringAttribute("fnchangedstate", &rr_state) != XML_SUCCESS) {
      Serial.println("fnchangedstate attribute not found or wrong type.");
      return;
    }
    Serial.println("fnchangedstate (raw): " + String(rr_state));
    if (strcmp(rr_state, "true")==0) {
      Serial.println("fnchangedstate: true");
      functionState = true;
    }
    else if (strcmp(rr_state, "false")==0) {
      Serial.println("fnchangedstate: false");
      functionState = false;
    }
    else {
      Serial.println("unknown fnchangedstate value - disregarding message.");
      return;
    }

    // set function pin
    if (functionState) {
      digitalWrite(functionPin, HIGH);
    } else {
      digitalWrite(functionPin, LOW);
    }
  }
}

// set the motor to a desired power.
void setTrainSpeed(int newTrainSpeed) {
  const int MAX_ARDUINO_POWER = 1023;

  int power = abs(map(newTrainSpeed, 0, maxTrainSpeed, 0, MAX_ARDUINO_POWER * maxTrainSpeed / 100));
  Serial.println("Setting motor speed: " + String(newTrainSpeed) + " (power: " + String(power) + ")");

  currentTrainSpeed = newTrainSpeed;
  int dir = currentTrainSpeed >= 0;

  if (MOTORSHIELD_TYPE == 1) {
    // motor shield type L298N
    if (dir ^ REVERSE_A) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    if (dir ^ REVERSE_B) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    } else {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }
    analogWrite(enA, power);
    analogWrite(enB, power);
  } else {
    // motor shield type L9110
    if (dir ^ REVERSE_A) {
      analogWrite(in1, 0);
      analogWrite(in2, power);
    } else {
      analogWrite(in1, power);
      analogWrite(in2, 0);
    }
    if (dir ^ REVERSE_B) {
      analogWrite(in3, 0);
      analogWrite(in4, power);
    } else {
      analogWrite(in3, power);
      analogWrite(in4, 0);
    }
  }
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


// gently adapt train speed (increase/decrease slowly)
void accelerateTrainSpeed() {
  if (currentTrainSpeed != targetTrainSpeed) {
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


void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  accelerateTrainSpeed();

  sendMQTTPing(&client, mqttClientName_char);
  sendMQTTBatteryLevel();
}
