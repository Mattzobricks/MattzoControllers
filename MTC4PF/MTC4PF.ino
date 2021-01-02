// MattzoTrainController for Power Functions Firmware
// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// ****************************************
// TARGET-PLATTFORM for this sketch: ESP-12
// ****************************************

// The following #include directives need some enum definitions before the compiler gets to them...
// MOTORSHIELD_TYPE represents the type motor shield that this controller uses.
// 1 = L298N, 2 = L9110, 3 = Lego IR Receiver 8884
// Types 1 and 2 are motorshields that are physically connected to the controller
// Commands to type 3 are transmitted via an infrared LED
enum struct MotorShieldType
{
  NONE = 0x0,
  L298N = 0x1,
  L9110 = 0x2,
  LEGO_IR_8884 = 0x3,
  WIFI_TRAIN_RECEIVER_4DBRIX = 0x4
};

// Min and max useful power for Arduino based motor shields
const int MIN_ARDUINO_POWER = 400;   // minimum useful arduino power. May be overwritten in motor shield configuration
const int MAX_ARDUINO_POWER = 1023;  // maximum arduino power

// Virtual pins for lights connected to Lego IR Receiver 8884.
// Function pins should only be set to this constant if MOTORSHIELD_TYPE == LEGO_IR_8884.
const uint8_t IR_LIGHT_RED = 254;
const uint8_t IR_LIGHT_BLUE = 255;

#define MATTZO_CONTROLLER_TYPE "MTC4PF"
#include <ESP8266WiFi.h>                // WiFi library for ESP-8266
#include "MattzoPowerFunctions.h"       // Power Functions library (required for LEGO Infrared Receiver 8884)

struct MattzoLocoConfiguration {
  String locoName;
  int locoAddress;
  int accelerationInterval;
  int accelerateStep;
  int brakeStep;
};

struct MattzoMotorShieldConfiguration {
  String motorShieldName;
  MotorShieldType motorShieldType;
  int minArduinoPower;
  int maxArduinoPower;
  int configMotorA;
  int configMotorB;
  int locoAddress;
};

// Forward declaration
class MattzoLoco;
class MattzoMotorShield;

// MattzoBricks library files
#include "MTC4PF_Configuration.h"       // this file should be placed in the same folder
#include "MattzoController_Library.h"   // this file needs to be placed in the Arduino library folder



// Class containing a locomotive object as defined in Rocrail
class MattzoLoco {
public:
  // Members
  String _locoName;                             // name of the loco as specified in Rocrail
  int _locoAddress;                             // address of the loco in Rocrail
  int _currentTrainSpeed = 0;                   // current speed of this train
  int _targetTrainSpeed = 0;                    // Target speed of this train
  int _maxTrainSpeed = 0;                       // Maximum speed of this train as configured in Rocrail
  unsigned long _lastAccelerate = millis();     // time of the last speed adjustment

  // Motor acceleration parameters
  int _accelerationInterval = 100;       // pause between individual speed adjustments in milliseconds
  int _accelerateStep = 1;               // acceleration increment for a single acceleration step
  int _brakeStep = 2;                    // brake decrement for a single braking step

  // Methods
  void initMattzoLoco(MattzoLocoConfiguration c) {
    _locoName = c.locoName;
    _locoAddress = c.locoAddress;
    _accelerationInterval = c.accelerationInterval;
    _accelerateStep = c.accelerateStep;
    _brakeStep = c.brakeStep;
  };

  String getNiceName() {
    return _locoName + " (" + _locoAddress + ")";
  }

  // sets a new target speed
  void setTargetTrainSpeed(int targetTrainSpeed) {
    _targetTrainSpeed = targetTrainSpeed;
    _lastAccelerate = millis() - _accelerationInterval;
  }
} myLocos[NUM_LOCOS];  // Objects for MattzoLocos


// Base class for motor shields
class MattzoMotorShield {
public:
  // Members
  String _motorShieldName;
  MotorShieldType _motorShieldType;
  int _minArduinoPower = MIN_ARDUINO_POWER;  // minimum useful power setting
  int _maxArduinoPower = MAX_ARDUINO_POWER;  // maximum useful power setting
  int _configMotorA = 0;  // 1 = forward, 0 not installed, -1 = reverse
  int _configMotorB = 0;
  int _locoAddress;   // address of the Rocrail loco in which this motor shield is built-in

  // Methods
  void initMattzoMotorShield(MattzoMotorShieldConfiguration c) {
    _motorShieldName = c.motorShieldName;
    _motorShieldType = c.motorShieldType;
    _minArduinoPower = c.minArduinoPower;
    _maxArduinoPower = c.maxArduinoPower;
    _configMotorA = c.configMotorA;
    _configMotorB = c.configMotorB;
    _locoAddress = c.locoAddress;
  }

  String getNiceName() {
    return _motorShieldName;
  }

  bool checkLocoAddress(int locoAddress) {
    return (locoAddress == 0 || locoAddress == _locoAddress);
  }
} myMattzoMotorShields[NUM_MOTORSHIELDS];



// Functions (lights)
bool functionCommand[NUM_FUNCTIONS];  // Desired state of a function
bool functionState[NUM_FUNCTIONS];    // Actual state of a function

enum struct LightEventType
{
  STOP = 0x0,
  FORWARD = 0x1,
  REVERSE = 0x2
};

// Power functions object for LEGO IR Receiver 8884
MattzoPowerFunctions powerFunctions(IR_LED_PIN, IR_CHANNEL);

// Report battery level
#define REPORT_BATTERYLEVEL false               // set to true or false to allow or omit battery level reports
const int SEND_BATTERYLEVEL_INTERVAL = 60000;   // interval for sending battery level in milliseconds
const int BATTERY_PIN = A0;
const int VOLTAGE_MULTIPLIER = 20000/5000 - 1;  // Rbottom = 5 kOhm; Rtop = 20 kOhm; => voltage split factor
const int MAX_AI_VOLTAGE = 5100;                // maximum analog input voltage on pin A0. Usually 5000 = 5V = 5000mV. Can be slightly adapted to correct small deviations
unsigned long lastBatteryLevelMsg = millis();   // Time of the last battery report

// Global emergency brake flag.
boolean ebreak = false;



void setup() {
  // load config from EEPROM, initialize Wifi, MQTT etc.
  setupMattzoController();

  mcLog("initializing function pins");

  // initialize function pins
  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    // only real (non-virtual) pins shall be initialized
    if (FUNCTION_PIN[i] < IR_LIGHT_RED) {
      pinMode(FUNCTION_PIN[i], OUTPUT);
    }
    functionCommand[i] = false;
    functionState[i] = false;
  }

  mcLog("loading loco and motor shield configuration");

  // load loco configuration
  MattzoLocoConfiguration* locoConf = getMattzoLocoConfiguration();
  for (int i = 0; i < NUM_LOCOS; i++) {
    myLocos[i].initMattzoLoco(*(locoConf + i));
  }

  // load motor shield configuration
  MattzoMotorShieldConfiguration* msConf = getMattzoMotorShieldConfiguration();
  for (int i = 0; i < NUM_MOTORSHIELDS; i++) {
    myMattzoMotorShields[i].initMattzoMotorShield(*(msConf + i));
  }

  mcLog("initializing built-in motor shield");

  // initialize pins for built-in motor shield (if any)
  switch (MOTORSHIELD_TYPE) {
    case MotorShieldType::L298N:
      // initialize motor pins for L298N
      pinMode(enA, OUTPUT);
      pinMode(enB, OUTPUT);
    case MotorShieldType::L9110:
      // initialize motor pins for L298N (continued) and L9110
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
      break;
    case MotorShieldType::LEGO_IR_8884:
      // Power Functions instance is initialized and mapped to the 
      // IR pin in the global section at the beginning of the code
      // => nothing to do here
      break;
    case MotorShieldType::WIFI_TRAIN_RECEIVER_4DBRIX:
      // init handler for 4DBrix WiFi Train Receiver
      // => nothing to do here
      break;
    default:
      ;
      // => nothing to do here
  }

  mcLog("stopping all trains");

  // stop all directly connected train motors
  for (int i=0; i < NUM_LOCOS; i++) {
    setTrainSpeed(0, i);
  }
}



int getMattzoLocoIndexByLocoAddress(int locoAddress) {
  // mcLog("getMattzoLocoIndexByLocoAddress is checking if this controller handles loco " + String(locoAddress) + "...");
  for (int l = 0; l < NUM_LOCOS; l++) {
    if (myLocos[l]._locoAddress == locoAddress) {
      return l;
    }
  }
  return -1;
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
    mcLog("Error parsing");
    return;
  }

  mcLog("Parsing XML successful");

  const char * rr_id = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";
  int rr_addr = 0;

  XMLElement *element;
  
  // check for lc message
  element = xmlDocument.FirstChildElement("lc");
  if (element != NULL) {
    mcLog("<lc> node found. Processing loco message...");

    // -> process lc (loco) message

    // query id attribute. This is the loco id.
    // The id is a mandatory field. If not found, the message is discarded.
    // Nevertheless, the id has no effect on the controller behaviour. Only the "addr" attribute is relevant for checking if the message is for this controller - see below.
    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      mcLog("id attribute not found or wrong type.");
      return;
    }
    mcLog("loco id: " + String(rr_id));
  
    // query addr attribute. This is the address of the loco as specified in Rocrail.
    // Must match the locoAddress of the train object.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog("addr attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("addr: " + String(rr_addr));

    int locoIndex = getMattzoLocoIndexByLocoAddress(rr_addr);
    if (locoIndex < 0) {
      mcLog("Message disregarded, as this controller does not handle train " + String(rr_addr));
      return;
    }
    MattzoLoco& loco = myLocos[locoIndex];
    mcLog("Consuming message for train " + loco.getNiceName());

    // query dir attribute. This is the direction information for the loco (forward, reverse)
    const char * rr_dir = "xxxxxx";  // expected values are "true" or "false"
    int dir;
    if (element->QueryStringAttribute("dir", &rr_dir) != XML_SUCCESS) {
      mcLog("dir attribute not found or wrong type.");
      return;
    }
    mcLog("dir (raw): " + String(rr_dir));
    if (strcmp(rr_dir, "true")==0) {
      mcLog("direction: forward");
      dir = 1;
    }
    else if (strcmp(rr_dir, "false")==0) {
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
    loco.setTargetTrainSpeed(rr_v * dir);
    loco._maxTrainSpeed = rr_vmax;
    mcLog("Message parsing complete, target speed set to " + String(loco._targetTrainSpeed) + " (current: " + String(loco._currentTrainSpeed) + ", max: " + String(loco._maxTrainSpeed) + ")");

    return;
  }

  // Check for fn message
  element = xmlDocument.FirstChildElement("fn");
  if (element != NULL) {
    mcLog("<fn> node found. Processing fn message...");

    // -> process fn (function) message

    // query id attribute. This is the loco name.
    // The id is a mandatory field. If not found, the message is discarded.
    if (element->QueryStringAttribute("id", &rr_id) != XML_SUCCESS) {
      mcLog("id attribute not found or wrong type.");
      return;
    }
    mcLog("function id: " + String(rr_id));

    // query addr attribute. This is the address of the loco as specified in Rocrail.
    // Must match the locoAddress of the train object.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog("addr attribute not found or wrong type. Message disregarded.");
      return;
    }
    mcLog("addr: " + String(rr_addr));

    int locoIndex = getMattzoLocoIndexByLocoAddress(rr_addr);
    if (locoIndex < 0) {
      mcLog("Message disregarded, as this controller does not handle train " + String(rr_addr));
      return;
    }
    MattzoLoco& loco = myLocos[locoIndex];
    mcLog("Consuming message for train " + loco.getNiceName());

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

    // Check if the function is associated with the loco
    if (FUNCTION_PIN_LOCO_ADDRESS[functionPinId] != loco._locoAddress) {
      mcLog("Function PIN is associated with loco " + String(FUNCTION_PIN_LOCO_ADDRESS[functionPinId]) + ", not with " + String(loco._locoAddress) + ". Disregarding message.");
      return;
    }

    // query fnchangedstate attribute. This is value if the function shall be set on or off
    const char * rr_state = "xxxxxx";  // expected values are "true" or "false"
    if (element->QueryStringAttribute("fnchangedstate", &rr_state) != XML_SUCCESS) {
      mcLog("fnchangedstate attribute not found or wrong type.");
      return;
    }
    mcLog("fnchangedstate (raw): " + String(rr_state));
    if (strcmp(rr_state, "true")==0) {
      mcLog("fnchangedstate: true");
      functionCommand[functionPinId] = true;
    }
    else if (strcmp(rr_state, "false")==0) {
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

    const char * rr_cmd = "-unknown--unknown--unknown--unknown--unknown--unknown--unknown-";

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

// set the motor(s) of a train to a desired speed
void setTrainSpeed(int newTrainSpeed, int locoIndex) {
  // Motorshield specific constants and variables
  const int MAX_IR_SPEED = 100;  // maximum speed to power functions speed mapping function
  MattzoPowerFunctionsPwm pfPWMRed;
  MattzoPowerFunctionsPwm pfPWMBlue;
  int irSpeed = 0;

  MattzoLoco& loco = myLocos[locoIndex];

  // Walk through all motor shields and check if they belong to the loco. If yes, set power!
  for (int i = 0; i < NUM_MOTORSHIELDS; i++) {
    if (myMattzoMotorShields[i].checkLocoAddress(loco._locoAddress)) {
      boolean dir = loco._currentTrainSpeed >= 0;  // true = forward, false = reverse
      int power;  // power level for all arduino based motor shields (L298N, L9110, 4DBrix)
    
      // Calculate arduino power
      if (newTrainSpeed != 0) {
        power = map(abs(newTrainSpeed), 0, loco._maxTrainSpeed, myMattzoMotorShields[i]._minArduinoPower, myMattzoMotorShields[i]._maxArduinoPower);
      }
      else {
        power = 0;
      }

      switch (myMattzoMotorShields[i]._motorShieldType) {
      case MotorShieldType::L298N:
        ;
      case MotorShieldType::L9110:
        // motor shield types L298N and L9110
        mcLog("Setting motor speed: " + String(newTrainSpeed) + " (power: " + String(power) + ") for motor shield " + myMattzoMotorShields[i].getNiceName());

        if (MOTORSHIELD_TYPE == MotorShieldType::L298N) {
          // motor shield type L298N
          if (myMattzoMotorShields[i]._configMotorA != 0) {
            if (dir ^ (myMattzoMotorShields[i]._configMotorA < 0)) {
              digitalWrite(in1, LOW);
              digitalWrite(in2, HIGH);
            }
            else {
              digitalWrite(in1, HIGH);
              digitalWrite(in2, LOW);
            }
            analogWrite(enA, power);
          }
          if (myMattzoMotorShields[i]._configMotorB != 0) {
            if (dir ^ (myMattzoMotorShields[i]._configMotorB < 0)) {
              digitalWrite(in3, LOW);
              digitalWrite(in4, HIGH);
            }
            else {
              digitalWrite(in3, HIGH);
              digitalWrite(in4, LOW);
            }
            analogWrite(enB, power);
          }
        }
        else if (MOTORSHIELD_TYPE == MotorShieldType::L9110) {
          // motor shield type L9110
          if (myMattzoMotorShields[i]._configMotorA != 0) {
            if (dir ^ (myMattzoMotorShields[i]._configMotorA < 0)) {
              analogWrite(in1, 0);
              analogWrite(in2, power);
            }
            else {
              analogWrite(in1, power);
              analogWrite(in2, 0);
            }
          }
          if (myMattzoMotorShields[i]._configMotorB != 0) {
            if (dir ^ (myMattzoMotorShields[i]._configMotorB < 0)) {
              analogWrite(in3, 0);
              analogWrite(in4, power);
            }
            else {
              analogWrite(in3, power);
              analogWrite(in4, 0);
            }
          }
        }

        break;

        // motor shield type Lego IR Receiver 8884
      case MotorShieldType::LEGO_IR_8884:
        if (loco._maxTrainSpeed > 0) {
          irSpeed = newTrainSpeed * MAX_IR_SPEED / loco._maxTrainSpeed;
        }
        pfPWMRed = powerFunctions.speedToPwm(IR_PORT_RED * irSpeed);
        pfPWMBlue = powerFunctions.speedToPwm(IR_PORT_BLUE * irSpeed);
        mcLog("Setting motor speed: " + String(newTrainSpeed) + " (IR speed: " + irSpeed + ")");

        if (IR_PORT_RED) {
          powerFunctions.single_pwm(MattzoPowerFunctionsPort::RED, pfPWMRed);
        }
        if (IR_PORT_BLUE) {
          powerFunctions.single_pwm(MattzoPowerFunctionsPort::BLUE, pfPWMBlue);
        }

        break;

        // motor shield type 4DBrix WiFi Train Receiver
      case MotorShieldType::WIFI_TRAIN_RECEIVER_4DBRIX:
        mcLog("Setting motor speed: " + String(newTrainSpeed) + " (power: " + String(power) + ") for 4DBrix WiFi Train Receiver " + myMattzoMotorShields[i].getNiceName());
        send4DMessage(power * myMattzoMotorShields[i]._configMotorA * dir, myMattzoMotorShields[i]._motorShieldName);

        break;

      } // of switch
    } // of if
  } // of for

  // Execute light events
  if (newTrainSpeed == 0 && loco._currentTrainSpeed != 0) {
    lightEvent(LightEventType::STOP, locoIndex);
  }
  else if (newTrainSpeed > 0 && loco._currentTrainSpeed <= 0) {
    lightEvent(LightEventType::FORWARD, locoIndex);
  }
  else if (newTrainSpeed < 0 && loco._currentTrainSpeed >= 0) {
    lightEvent(LightEventType::REVERSE, locoIndex);
  }

  loco._currentTrainSpeed = newTrainSpeed;
}

// gently adapt train speed (increase/decrease slowly)
void accelerateTrainSpeed() {
  boolean accelerateFlag;
  int step;
  int nextSpeed;

  for (int locoIndex = 0; locoIndex < NUM_LOCOS; locoIndex++) {
    MattzoLoco& loco = myLocos[locoIndex];

    if (ebreak) {
      // emergency break pulled and train moving -> stop train immediately
      if (loco._currentTrainSpeed != 0) {
        setTrainSpeed(0, locoIndex);
      }
    }
    else if (loco._currentTrainSpeed != loco._targetTrainSpeed) {
      if (loco._targetTrainSpeed == 0) {
        // stop -> execute immediately
        setTrainSpeed(0, locoIndex);
      }
      else if (millis() - loco._lastAccelerate >= loco._accelerationInterval) {
        loco._lastAccelerate = millis();

        // determine if trains accelerates or brakes
        accelerateFlag = abs(loco._currentTrainSpeed) < abs(loco._targetTrainSpeed) && (loco._currentTrainSpeed * loco._targetTrainSpeed > 0);
        step = accelerateFlag ? loco._accelerateStep : loco._brakeStep;

        // accelerate / brake gently
        if (loco._currentTrainSpeed < loco._targetTrainSpeed) {
          nextSpeed = min(loco._currentTrainSpeed + step, loco._targetTrainSpeed);
        }
        else {
          nextSpeed = max(loco._currentTrainSpeed - step, loco._targetTrainSpeed);
        }
        setTrainSpeed(nextSpeed, locoIndex);
      }
    }
  }
}

// execute light event
// IF DESIRED, YOU MAY UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
void lightEvent(LightEventType le, int locoIndex) {
  if (!AUTO_LIGHTS)
    return;
  
  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    if (locoIndex == 0 || locoIndex == FUNCTION_PIN_LOCO_ADDRESS[i]) {
      switch (le) {
      case LightEventType::STOP:
        mcLog("Light event stop");
        // switch all functions off
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        functionCommand[i] = false;
        break;
      case LightEventType::FORWARD:
        mcLog("Light event forward");
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        functionCommand[i] = (i % 2) == 0;
        break;
      case LightEventType::REVERSE:
        mcLog("Light event reverse");
        // UPDATE THIS CODE SO THAT IT FITS YOUR NEEDS!
        functionCommand[i] = (i % 2) == 1;
        break;
      }
    }
  }
}

// switch lights on or off
void setLights() {
  for (int i = 0; i < NUM_FUNCTIONS; i++) {
    bool onOff = functionCommand[i];

    if (ebreak) {
      // override function state on ebreak (alternate lights on/off every 500ms)
      long phase = (millis() / 500) % 2;
      onOff = (phase + i) % 2 == 0;
    }

    if (functionState[i] != onOff) {
      functionState[i] = onOff;
      mcLog("Flipping function " + String(i + 1) + " to " + String(onOff));

      MattzoPowerFunctionsPwm irPwmLevel = onOff ? MattzoPowerFunctionsPwm::FORWARD7 : MattzoPowerFunctionsPwm::BRAKE;

      switch (FUNCTION_PIN[i]) {
        case IR_LIGHT_RED:
          powerFunctions.single_pwm(MattzoPowerFunctionsPort::RED, irPwmLevel);
          break;
        case IR_LIGHT_BLUE:
          powerFunctions.single_pwm(MattzoPowerFunctionsPort::BLUE, irPwmLevel);
          break;
        default:
          digitalWrite(FUNCTION_PIN[i], onOff ? HIGH : LOW);
      } // of switch
    } // of if
  } // of for
}

// Send battery level to mqtt
// Not really useful at the moment, but might be relevant in further version of the MattzoController firmware
void sendBatteryLevel2MQTT() {
  if (REPORT_BATTERYLEVEL && mqttClient.connected()) {
    if (millis() - lastBatteryLevelMsg >= SEND_BATTERYLEVEL_INTERVAL) {
      lastBatteryLevelMsg = millis();
      int a0Value = analogRead(BATTERY_PIN);
      int voltage = map(a0Value, 0, 1023, 0, MAX_AI_VOLTAGE * VOLTAGE_MULTIPLIER);  // Battery Voltage in mV (Millivolt)
      if (voltage > 99999) voltage = 99999;

      mcLog("sending battery level raw=" + String(a0Value) + ", mv=" + String(voltage));
      String batteryMessage = mattzoControllerName + " raw=" + String(a0Value) + ", mv=" + String(voltage);
      char batteryMessage_char[batteryMessage.length() + 1];  // client name + 5 digits for voltage in mV plus terminating char
      batteryMessage.toCharArray(batteryMessage_char, batteryMessage.length() + 1);

      mqttClient.publish("roc2bricks/battery", batteryMessage_char);
    }
  }
}

// sends a message to a 4DBrix WiFi Train Receiver
// - power is a value that indicates the desired power of the train motor. Useful values range from -1023 (full reverse) to 1023 (full forward). 0 means "stop".
// - locoName is the name of the loco as chosen in the 4DBrix WiFi Train Receiver
void send4DMessage(int power, String locoName) {
  if (getConnectionStatus() == MCConnectionStatus::CONNECTED) {
    // MQTT topic has max. 60 chars, so max. characters is LOCO_NAME is 50.
    const int MAX_MQTT_TOPIC_LENGTH = 60;
    char mqttTopicCharArray[MAX_MQTT_TOPIC_LENGTH];

    // Prepare the 4-byte 4DBrix command string for train controller motor commands
    // First character is always "D" (0x44).
    char* mqttCommandBytes = "D\"!!";  // default ("stop!")
    int powerValue4DBrix = 0;

    // Second character indicates the direction of the motor
    if (power > 0)
    {
      // forward
      mqttCommandBytes[1] = 0x32;  // = "2"
    }
    else
    {
      // backward
      mqttCommandBytes[1] = 0x2A;  // = "*"
    }

    // Third and fourth character contain the motor speed
    powerValue4DBrix = min(abs(power), 1023);   // limit range to value between 0 to 1023.

    mqttCommandBytes[2] = (byte)(powerValue4DBrix / 64 + 33);
    mqttCommandBytes[3] = (byte)(powerValue4DBrix % 64 + 33);

    // send MQTT command to MQTT broker
    // mcLog("Sending 4DBrix command " + String(mqttCommandBytes) + " for loco " + locoName + ", power " + String(powerValue4DBrix));
    String mqttTopicString = "nControl/" + String(locoName);
    mqttTopicString.toCharArray(mqttTopicCharArray, MAX_MQTT_TOPIC_LENGTH);
    mqttClient.publish(mqttTopicCharArray, mqttCommandBytes);
  }
}

void loop() {
  loopMattzoController();
  accelerateTrainSpeed();
  setLights();
  sendBatteryLevel2MQTT();
}
