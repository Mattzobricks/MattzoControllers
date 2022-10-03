// MattzoTrainController for Power Functions Firmware
// Author: Dr. Matthias Runte
// Copyright 2020, 2021 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// ******************************************
// TARGET-PLATTFORM for this sketch: ESP-8266
// ******************************************
#include "MTC.h"
#include "MTC4PF.h"

#include "../conf/default/network_config.h"
#include "../conf/default/controller_config.h"

// Array of loco classes
class MattzoLoco myLocos[NUM_LOCOS];
// Array of motor shield classes
class MattzoMotorShield myMattzoMotorShields[NUM_MOTORSHIELDS];

// Power functions objects for LEGO IR Receiver 8884
MattzoPowerFunctions powerFunctions0(IR_LED_PIN, 0);
MattzoPowerFunctions powerFunctions1(IR_LED_PIN, 1);
MattzoPowerFunctions powerFunctions2(IR_LED_PIN, 2);
MattzoPowerFunctions powerFunctions3(IR_LED_PIN, 3);

// Array of train light objects
struct TrainLight trainLight[NUM_TRAIN_LIGHTS];

// Time of last battery level report message
unsigned long lastBatteryLevelMsg = millis();   // Time of the last battery report

// Global emergency brake flag
boolean ebreak = false;



void setup() {
  // load config from EEPROM, initialize Wifi, MQTT etc.
  setupMattzoController(false);

  // initialize train light output pins
  mcLog("initializing train light pins");
  for (int tl = 0; tl < NUM_TRAIN_LIGHTS; tl++) {
    if (trainLightConfiguration[tl].trainLightType == TrainLightType::ESP_OUTPUT_PIN) {
      pinMode(trainLightConfiguration[tl].pin, OUTPUT);
    }
  }

  // load loco configuration
  mcLog("loading loco configuration");
  MattzoLocoConfiguration* locoConf = getMattzoLocoConfiguration();
  for (int i = 0; i < NUM_LOCOS; i++) {
    myLocos[i].initMattzoLoco(*(locoConf + i));
  }

  // load motor shield configuration
  mcLog("loading motor shield configuration");
  MattzoMotorShieldConfiguration* msConf = getMattzoMotorShieldConfiguration();
  for (int i = 0; i < NUM_MOTORSHIELDS; i++) {
    myMattzoMotorShields[i].initMattzoMotorShield(*(msConf + i));

    // initialize pins of directly wired motor shields (if any)
    switch (myMattzoMotorShields[i]._motorShieldType) {
      case MotorShieldType::L298N:
        // initialize motor pins for L298N
        pinMode(myMattzoMotorShields[i]._L298N_enA, OUTPUT);
        pinMode(myMattzoMotorShields[i]._L298N_enB, OUTPUT);
        // fall through!
      case MotorShieldType::L9110:
        // initialize motor pins for L298N (continued) and L9110
        pinMode(myMattzoMotorShields[i]._in1, OUTPUT);
        pinMode(myMattzoMotorShields[i]._in2, OUTPUT);
        pinMode(myMattzoMotorShields[i]._in3, OUTPUT);
        pinMode(myMattzoMotorShields[i]._in4, OUTPUT);
        break;
      case MotorShieldType::LEGO_IR_8884:
        // nothing to do here
        break;
      default:
        ;
    }
  }

  // stop all locos
  mcLog("stopping all locos");
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
  for (unsigned int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  msg[length] = '\0';

  // mcLog("Received MQTT message [" + String(topic) + "]: " + String(msg));

  XMLDocument xmlDocument;
  if(xmlDocument.Parse(msg)!= XML_SUCCESS){
    mcLog("Error parsing");
    return;
  }

  // mcLog("Parsing XML successful");

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
    mcLog2("Received fn message...", LOG_DEBUG);

    // -> process fn (function) message

    // query addr attribute. This is the address of the loco as specified in Rocrail.
    // Must match the locoAddress of the train object.
    if (element->QueryIntAttribute("addr", &rr_addr) != XML_SUCCESS) {
      mcLog2("addr attribute not found or wrong type. Message disregarded.", LOG_DEBUG);
      return;
    }
    mcLog2("addr: " + String(rr_addr), LOG_DEBUG);

    // query fnchanged attribute. This is information which function shall be set.
    int rr_functionNo;
    if (element->QueryIntAttribute("fnchanged", &rr_functionNo) != XML_SUCCESS) {
      // if fnchanged attribute not found -> f0 was changed.
      rr_functionNo = 0;
    }
    mcLog2("fnchanged: f" + String(rr_functionNo), LOG_DEBUG);

    // query fnchangedstate attribute. This is value if the function shall be set on or off
    const char * rr_state_String = "xxxxxx";  // expected values are "true" or "false"
    bool rr_state;
    if (rr_functionNo > 0) {
      if (element->QueryStringAttribute("fnchangedstate", &rr_state_String) != XML_SUCCESS) {
        mcLog2("fnchangedstate attribute not found or wrong type.", LOG_DEBUG);
        return;
      }
    } else {
      if (element->QueryStringAttribute("f0", &rr_state_String) != XML_SUCCESS) {
        mcLog2("f0 attribute not found or wrong type.", LOG_DEBUG);
        return;
      }
    }
    if (strcmp(rr_state_String, "true")==0) {
      mcLog2("fnchangedstate: true", LOG_DEBUG);
      rr_state = true;
    }
    else if (strcmp(rr_state_String, "false")==0) {
      mcLog2("fnchangedstate: false", LOG_DEBUG);
      rr_state = false;
    }
    else {
      mcLog2("unknown fnchangedstate value - disregarding message.", LOG_DEBUG);
      return;
    }

    mcLog2("Received fn message: loco address " + String(rr_addr) + ", fn" + String(rr_functionNo) + ", state=" + String(rr_state), LOG_DEBUG);
    handleRocrailFunction(rr_addr, rr_functionNo, rr_state);

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

  // mcLog("Unknown message, disregarded.");
}

// set all motors of a train to a desired power level
void setTrainSpeed(int newTrainSpeed, int locoIndex) {
  int desiredDirection;  // Desired direction. forward = 1, reverse = -1
  int desiredPowerLevel;    // Desired power level (0..MAX_ARDUINO_POWER). Range from min to max is identical for all motor shield types. Does not include the direction (absolute value)
  int desiredPower;         // Desired power (-MAX_ARDUINO_POWER .. MAX_ARDUINO_POWER). Range is identical for all motor shield types. Includes the direction (+/-).
  MattzoLoco& loco = myLocos[locoIndex];

  // Determine desired direction.
  desiredDirection = (newTrainSpeed >= 0) ? 1 : -1;

  // Walk through all motor shields and check if they belong to the loco. If yes, set power!
  for (int motorShieldIndex = 0; motorShieldIndex < NUM_MOTORSHIELDS; motorShieldIndex++) {
    if (myMattzoMotorShields[motorShieldIndex].checkLocoAddress(loco._locoAddress)) {
      // Calculate desired power level for motor shield ports
      if (newTrainSpeed != 0) {
        desiredPowerLevel = map(abs(newTrainSpeed), 0, loco._maxTrainSpeed, myMattzoMotorShields[motorShieldIndex]._minArduinoPower, myMattzoMotorShields[motorShieldIndex]._maxArduinoPower);
      }
      else {
        desiredPowerLevel = 0;
      }
      
      // Calculate desired power
      desiredPower = desiredDirection * desiredPowerLevel;

      // Set power levels on motor shield ports if they are configured for a motor
      mcLog("Setting train speed " + String(newTrainSpeed) + " (power: " + String(desiredPower) + ") for motor shield index " + String(motorShieldIndex));
      if (myMattzoMotorShields[motorShieldIndex]._configMotorA) {
        setMotorShieldPower(motorShieldIndex, 0, desiredPower * myMattzoMotorShields[motorShieldIndex]._configMotorA);
      }
      if (myMattzoMotorShields[motorShieldIndex]._configMotorB) {
        setMotorShieldPower(motorShieldIndex, 1, desiredPower * myMattzoMotorShields[motorShieldIndex]._configMotorB);
      }
    } // of if
  } // of for

  // Trigger light events
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

void setMotorShieldPower(int motorShieldIndex, int motorPortIndex, int desiredPower) {
  // Motorshield unspecific constants and variables
  int desiredPowerLevel = abs(desiredPower);
  int desiredDirection = (desiredPower >= 0) ? 1 : -1;
  bool directionIsForward = desiredDirection >= 0;

  // Motorshield specific constants and variables
  int irPowerLevel = 0;   // Power level for the LEGO IR Receiver 8884

  mcLog("setMotorShieldPower() called with msi=" + String(motorShieldIndex) + ", mpi=" + String(motorPortIndex) + ", desiredPower=" + String(desiredPower));

  switch (myMattzoMotorShields[motorShieldIndex]._motorShieldType) {
  case MotorShieldType::L298N:
    // motor shield type L298N
    // The preferred option to flip direction is to go via HIGH/HIGH on the input pins (set HIGH first, then LOW)
    if (motorPortIndex == 0) {
      if (directionIsForward) {
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in2, HIGH);
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in1, LOW);
      }
      else {
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in1, HIGH);
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in2, LOW);
      }
      analogWrite(myMattzoMotorShields[motorShieldIndex]._L298N_enA, desiredPowerLevel);
    }
    if (motorPortIndex == 1) {
      if (directionIsForward) {
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in4, HIGH);
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in3, LOW);
      }
      else {
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in3, HIGH);
        digitalWrite(myMattzoMotorShields[motorShieldIndex]._in4, LOW);
      }
      analogWrite(myMattzoMotorShields[motorShieldIndex]._L298N_enB, desiredPowerLevel);
    }
    break;

  case MotorShieldType::L9110:
    // motor shield type L9110
    if (motorPortIndex == 0) {
      if (directionIsForward) {
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in1, 0);
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in2, desiredPowerLevel);
      }
      else {
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in2, 0);
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in1, desiredPowerLevel);
      }
    }
    if (motorPortIndex == 1) {
      if (directionIsForward) {
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in3, 0);
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in4, desiredPowerLevel);
      }
      else {
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in4, 0);
        analogWrite(myMattzoMotorShields[motorShieldIndex]._in3, desiredPowerLevel);
      }
    }
    break;

  case MotorShieldType::LEGO_IR_8884:
    // motor shield type Lego IR Receiver 8884
    irPowerLevel = desiredPower * MAX_IR_POWERVALUE / MAX_ARDUINO_POWER;
    if (motorPortIndex == 0) {
      myMattzoMotorShields[motorShieldIndex].pfPowerLevelRed = powerFunctions0.speedToPwm(irPowerLevel);
    }
    else if (motorPortIndex == 1) {
      myMattzoMotorShields[motorShieldIndex].pfPowerLevelBlue = powerFunctions0.speedToPwm(irPowerLevel);
    }
    mcLog("Setting IR channel " + String(myMattzoMotorShields[motorShieldIndex]._irChannel) + " port " + String(motorPortIndex) + " to " + String(irPowerLevel));
    // Force immediate IR transmission
    transmitIRCommandsImmediate(motorShieldIndex);
    break;

    case MotorShieldType::NONE: // fall through
    default:
    // do nothing
    break;
  } // of switch
}

void transmitIRCommandsRoutine() {
  static unsigned long lastIRTransmission = 0;
  static int nextMotorShieldIndex = 0;

  if (millis() < lastIRTransmission + WAIT_BETWEEN_IR_TRANSMISSIONS_MS)
    return;

  lastIRTransmission = millis();
  nextMotorShieldIndex = transmitIRCommandsImmediate(nextMotorShieldIndex);
}

int transmitIRCommandsImmediate(int nextMotorShieldIndex) {
  for (int i = 0; i < NUM_MOTORSHIELDS; i++) {
    int motorShieldIndex = (i + nextMotorShieldIndex) % NUM_MOTORSHIELDS;
    if (myMattzoMotorShields[motorShieldIndex]._motorShieldType == MotorShieldType::LEGO_IR_8884) {
      // red port
      if (myMattzoMotorShields[motorShieldIndex]._configMotorA || myMattzoMotorShields[motorShieldIndex]._configMotorB) {
        if (myMattzoMotorShields[motorShieldIndex]._configMotorA) {
          // mcLog("Transmitting IR power for motor shield " + String(motorShieldIndex) + ", CH" + String(myMattzoMotorShields[motorShieldIndex]._irChannel) + "/RED...");
          switch (myMattzoMotorShields[motorShieldIndex]._irChannel) {
            case 0:
              powerFunctions0.single_pwm(MattzoPowerFunctionsPort::RED, myMattzoMotorShields[motorShieldIndex].pfPowerLevelRed);
              break;
            case 1:
              powerFunctions1.single_pwm(MattzoPowerFunctionsPort::RED, myMattzoMotorShields[motorShieldIndex].pfPowerLevelRed);
              break;
            case 2:
              powerFunctions2.single_pwm(MattzoPowerFunctionsPort::RED, myMattzoMotorShields[motorShieldIndex].pfPowerLevelRed);
              break;
            case 3:
              powerFunctions3.single_pwm(MattzoPowerFunctionsPort::RED, myMattzoMotorShields[motorShieldIndex].pfPowerLevelRed);
          }
        }
        // blue port
        if (myMattzoMotorShields[motorShieldIndex]._configMotorB) {
          // mcLog("Transmitting IR power for motor shield " + String(motorShieldIndex) + ", CH" + String(myMattzoMotorShields[motorShieldIndex]._irChannel) + "/BLUE...");
          switch (myMattzoMotorShields[motorShieldIndex]._irChannel) {
            case 0:
              powerFunctions0.single_pwm(MattzoPowerFunctionsPort::BLUE, myMattzoMotorShields[motorShieldIndex].pfPowerLevelBlue);
              break;
            case 1:
              powerFunctions1.single_pwm(MattzoPowerFunctionsPort::BLUE, myMattzoMotorShields[motorShieldIndex].pfPowerLevelBlue);
              break;
            case 2:
              powerFunctions2.single_pwm(MattzoPowerFunctionsPort::BLUE, myMattzoMotorShields[motorShieldIndex].pfPowerLevelBlue);
              break;
            case 3:
              powerFunctions3.single_pwm(MattzoPowerFunctionsPort::BLUE, myMattzoMotorShields[motorShieldIndex].pfPowerLevelBlue);
          }
        }
        nextMotorShieldIndex = motorShieldIndex + 1;
        return nextMotorShieldIndex;
      }
    }
  }
  return nextMotorShieldIndex;
}

String redBlueStringByIRPort(MattzoPowerFunctionsPort port) {
  if (port == MattzoPowerFunctionsPort::RED)
    return "red";
  else
    return "blue";
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

// execute light events
void lightEvent(LightEventType le, int locoIndex) {
  int locoAddress = myLocos[locoIndex]._locoAddress;

  for (int tlt_index = 0; tlt_index < NUM_TRAIN_LIGHT_TRIGGERS; tlt_index++) {
    if (locoAddress == 0 || trainLightTriggerConfiguration[tlt_index].locoAddress == locoAddress) {
      if (trainLightTriggerConfiguration[tlt_index].lightEventType == le) {
        switch (le) {
        case LightEventType::STOP:
          mcLog("Executing light event stop");
          break;
        case LightEventType::FORWARD:
          mcLog("Executing light event forward");
          break;
        case LightEventType::REVERSE:
          mcLog("Executing light event reverse");
          break;
        }

        setTrainLightState(
          trainLightTriggerConfiguration[tlt_index].trainLightIndex,
          trainLightTriggerConfiguration[tlt_index].trainLightStatus
        );
      }
    }
  }
}

// handle a rocrail function message
setTrainLightState(int locoAddress, int fnNo, bool fnOnOff) {
  bool mapperFound = false;

  for (int fm_index = 0; fm_index < NUM_FUNCTION_MAPPINGS; fm_index++) {
    if (locoFunctionMappingConfiguration[fm_index].locoAddress == locoAddress) {
      if (locoFunctionMappingConfiguration[fm_index].fnNo == fnNo) {
        if (locoFunctionMappingConfiguration[fm_index].fnOnOff == fnOnOff) {
          setTrainLightState(
            locoFunctionMappingConfiguration[fm_index].trainLightIndex,
            locoFunctionMappingConfiguration[fm_index].trainLightStatus
          );
          mapperFound = true;
        }
      }
    }
  }
  if (!mapperFound)
    mcLog("No action found for loco address " + String(locoAddress) + ", fn" + String(fnNo) + ", state=" + String(fnOnOff));
}

// Return train light status string (for debugging reasons)
String getTrainLightStatusString(TrainLightStatus trainLightStatus) {
  switch (trainLightStatus) {
    case TrainLightStatus::OFF:
      return ("off");
    case TrainLightStatus::ON:
      return ("on");
    case TrainLightStatus::FLASH:
      return ("flash");
    case TrainLightStatus::BLINK:
      return ("blink");
    default:
      return ("undefined - check getTrainLightStatusString()");
  }
}

// set a train light
void setTrainLightState(int trainLightIndex, TrainLightStatus trainLightStatus) {
  if (trainLightIndex < 0 || trainLightIndex >= NUM_TRAIN_LIGHTS) {
    mcLog("ERROR in setTrainLightState(): trainLightIndex out of bounds (" + String(trainLightIndex) + ")");
    return;
  }

  mcLog("Setting train light index " + String(trainLightIndex) + " to status " + getTrainLightStatusString(trainLightStatus));
  trainLight[trainLightIndex].desiredTrainLightState = trainLightStatus;
}

#define EBREAK_HALFPERIOD_MS 500
#define FLASH_HALFPERIOD_MS 750
#define BLINK_PERIOD_MS 2000

// switch lights on or off
void setLights() {
  for (int tl = 0; tl < NUM_TRAIN_LIGHTS; tl++) {
    TrainLightStatus desiredState = trainLight[tl].desiredTrainLightState;
    int actualPowerLevel = trainLight[tl].actualPowerLevel;
    
    // if emergency brake is active, invert light intensity every FLASH_HALFPERIOD_MS ms.
    bool ebreakModifier = false;
    if (ebreak) {
      // override function state on ebreak (alternate lights on/off every 500ms)
      int ebreakPhase = (millis() / EBREAK_HALFPERIOD_MS) % 2;
      ebreakModifier = (ebreakPhase + tl) % 2 == 0;
      // in case of flash or blink state, just flash
      if (desiredState == TrainLightStatus::FLASH || desiredState == TrainLightStatus::BLINK)
        desiredState = TrainLightStatus::ON;
    }

    // determine desiredPowerLevel by train light state and phase
    int desiredPowerLevel = trainLightConfiguration[tl].powerLevelOff;
    int flashPhase;
    bool flashModifier;
    int blinkTimeElapsed_ms;
    switch (desiredState) {
      case TrainLightStatus::OFF:
        desiredPowerLevel = ebreakModifier ? trainLightConfiguration[tl].powerLevelOn : trainLightConfiguration[tl].powerLevelOff;
        break;
      case TrainLightStatus::ON:
        desiredPowerLevel = ebreakModifier ? trainLightConfiguration[tl].powerLevelOff : trainLightConfiguration[tl].powerLevelOn;
        break;
      case TrainLightStatus::FLASH:
        flashPhase = (millis() / FLASH_HALFPERIOD_MS) % 2;
        flashModifier = flashPhase % 2 == 0;
        desiredPowerLevel = flashModifier ? trainLightConfiguration[tl].powerLevelOn : trainLightConfiguration[tl].powerLevelOff;
        break;
      case TrainLightStatus::BLINK:
        // ms elapsed since beginning of current blink phase
        blinkTimeElapsed_ms = millis() % BLINK_PERIOD_MS;
        desiredPowerLevel =
          trainLightConfiguration[tl].powerLevelOff + 
          (trainLightConfiguration[tl].powerLevelOn - trainLightConfiguration[tl].powerLevelOff) * abs(blinkTimeElapsed_ms - BLINK_PERIOD_MS / 2) / (BLINK_PERIOD_MS / 2);
        break;
      default:
        ;
    }

    // change light intensity if required
    if (desiredPowerLevel != actualPowerLevel) {
      switch (trainLightConfiguration[tl].trainLightType) {
        case TrainLightType::ESP_OUTPUT_PIN:
          mcLog("Setting output pin " + String(trainLightConfiguration[tl].pin) + " to " + String(desiredPowerLevel));
          // Scale 0..100 -> 0..MAX_ARDUINO_POWER
          analogWrite(trainLightConfiguration[tl].pin, desiredPowerLevel);
          break;
        case TrainLightType::POWER_FUNCTIONS:
          mcLog("Setting power functions, motor shield index " + String(trainLightConfiguration[tl].motorShieldIndex) + ", motor port index " + String(trainLightConfiguration[tl].motorPortIndex) + " to " + String(desiredPowerLevel));
          setMotorShieldPower(trainLightConfiguration[tl].motorShieldIndex, trainLightConfiguration[tl].motorPortIndex, desiredPowerLevel);
          break;
        default:
          ;
      } // of switch

      trainLight[tl].actualPowerLevel = desiredPowerLevel;
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
      String batteryMessage = String(MC_HOSTNAME) + " raw=" + String(a0Value) + ", mv=" + String(voltage);
      char batteryMessage_char[batteryMessage.length() + 1];  // client name + 5 digits for voltage in mV plus terminating char
      batteryMessage.toCharArray(batteryMessage_char, batteryMessage.length() + 1);

      mqttClient.publish("roc2bricks/battery", batteryMessage_char);
    }
  }
}

void loop() {
  loopMattzoController();
  accelerateTrainSpeed();
  transmitIRCommandsRoutine();
  setLights();
  sendBatteryLevel2MQTT();
}
