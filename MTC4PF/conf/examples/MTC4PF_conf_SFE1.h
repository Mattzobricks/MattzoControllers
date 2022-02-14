// Author: Dr. Matthias Runte
// Copyright 2020 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// This file needs to stay in the folder of the firmware for the specific MattzoController!

// Handling of different configurations:
//		It's a good idea to create copies for all the controllers of this type that you own.
//		Before compiling and uploading the firmware, just include the correct configuration into the firmware code.
//		This allows to manage specific configurations for the different controllers easily.

// Best practice:
// 1. Create a copy of this file if required (see above).
// 2. Go through the settings below and update the settings as required.



// *********************************************************************************************
// Example file for configuring the MTC4PF to control a train with L9110 motor shield and lights
// *********************************************************************************************


// *****
// LOCOS
// *****

// Number of locos (aka. MattzoLocos) controlled by this controller
const int NUM_LOCOS = 1;

// List of MattzoLocos
// The parameters have the following meaning:
// - locoName: name of the loco as setup in Rocrail
// - locoAddress: address of the loco as setup in Rocrail
// - accelerationInterval: time interval for acceleration / braking (default: 100 ms)
// - accelerateStep: power increment for each acceleration step
// - brakeStep: : power decrement for each braking step
MattzoLocoConfiguration* getMattzoLocoConfiguration() {
  static MattzoLocoConfiguration locoConf[NUM_LOCOS];

  locoConf[0] = (MattzoLocoConfiguration) {
    .locoName = "SFE",
    .locoAddress = 10020,
    .accelerationInterval = 100,
    .accelerateStep = 2,
    .brakeStep = 10
  };

  return locoConf;
}


// *************
// MOTOR SHIELDS
// *************
// Motor shields are usually electronical components attached to the MattzoTrainController.
// They are controlled via PWM signals from the controller and handle the higher currents required
// for train motors.
// Beside those physically existing motor shields, they are also partly or completely virtual motor shields as:
// - LEGO_IR_8884, and
// - WIFI_TRAIN_RECEIVER_4DBRIX
// It is important to note that one train can have MULTIPLE motor shields attached to it. This is relevant for the following scenarios:
// - Infrared LED controls multiple motors in a single train
// - More than one 4DBrix WiFi Train Receiver installed in a single train.
// - Combining different motorshields into a single train is also possible (e.g. steam loco with 4DBrix receiver in the front 
//   and an additional "push" waggon in the middle of the train with an L9110 motorshield).

// Number of motor shields controlled by this controller
const int NUM_MOTORSHIELDS = 1;

// List of motor shields that are controlled by this controller
// The parameters have the following meaning:
// - locoAddress: loco that this motor shields is attached to
// - motorShieldType: motor shield type
// - L298N_enA, L298N_enB: PWM signal pin for motor A / B, if L298N is used.
// - in1..in4: pin for motor direction control for motor shields L298N and L9110 (in1: forward motor A, in2: reverse motor A, in3: forward motor B, in4: reverse motor B).
// - minArduinoPower: minimum power (should be 0 for LEGO IR Receiver 8884)
// - maxArduinoPower: maximum power (max. 1023)
// - configMotorA: turning direction of motor A (1 = forward, -1 = backward, 0 = unused). In case of LEGO IR Receiver 8884, this is the motor connected to the red port.
// - configMotorB: same for motor B; if IR receiver: blue port
// - irChannel: if a LEGO IR Receiver 8884 is used, the selected channel of the receiver. May be 0, 1, 2 or 3. If the loco uses multiple IR receivers on different channels, additional motor shields for the loco are required.
MattzoMotorShieldConfiguration* getMattzoMotorShieldConfiguration() {
  static MattzoMotorShieldConfiguration msConf[NUM_MOTORSHIELDS];

  msConf[0] = (MattzoMotorShieldConfiguration) {
      .locoAddress = 10020,
      .motorShieldType = MotorShieldType::L9110,
      .L298N_enA = 0,
      .L298N_enB = 0,
      .in1 = D1,
      .in2 = D2,
      .in3 = 0,
      .in4 = 0,
      .minArduinoPower = MIN_ARDUINO_POWER,
      .maxArduinoPower = MAX_ARDUINO_POWER,
      .configMotorA = -1,
      .configMotorB = 0,
      .irChannel = -1
  };

  return msConf;
}


// *************************
// TRAIN LIGHT CONFIGURATION
// *************************

// Number of train lights controlled by this controller
#define NUM_TRAIN_LIGHTS 3

// List of train lights including their configuration
struct TrainLightConfiguration {
  // Train light type. Either some light that is directly wired to the ESP, or a light connected to a Power Functions motor port
  TrainLightType trainLightType;
  // If light is directly wired to the ESP, the output pin; else: unused (set to -1)
  uint8_t pin;
  // If light in connected to motor shield: index of the motor shield. Usually 0 (the first and only one). Else: unused (set to -1)
  int motorShieldIndex;
  // If light in connected to motor shield: port A: 0; port B: 1. Else: unused (set to -1)
  // ATTENTION: if a light is connected to a motor port, the corresponding configMotorA / configMotorB parameter in the motorshield configuration must be 0!
  int motorPortIndex;
  // light intensity (0..MAX_ARDUINO_POWER) if light is switched off. Usually = 0.
  int powerLevelOff;
  // light intensity (0..MAX_ARDUINO_POWER) if light is switched on. Full bright = MAX_ARDUINO_POWER = 1023.
  int powerLevelOn;
} trainLightConfiguration[NUM_TRAIN_LIGHTS] =
{
  {
    // 0: head light / red component
    .trainLightType = TrainLightType::ESP_OUTPUT_PIN,
    .pin = D5,
    .motorShieldIndex = 0,
    .motorPortIndex = -1,
    .powerLevelOff = 0,
    .powerLevelOn = 300,
  },
  {
    // 1: head light / green component
    .trainLightType = TrainLightType::ESP_OUTPUT_PIN,
    .pin = D6,
    .motorShieldIndex = 0,
    .motorPortIndex = -1,
    .powerLevelOff = 0,
    .powerLevelOn = 600,
  },
  {
    // 2: head light / blue component
    .trainLightType = TrainLightType::ESP_OUTPUT_PIN,
    .pin = D7,
    .motorShieldIndex = 0,
    .motorPortIndex = -1,
    .powerLevelOff = 0,
    .powerLevelOn = MAX_ARDUINO_POWER,
  },
};


// ******************************
// FUNCTION MAPPING CONFIGURATION
// ******************************

// Rocrail functions are used to MANUALLY switch train lights on and off

// Number of function mappings
#define NUM_FUNCTION_MAPPINGS 9

// List of function mappings
struct LocoFunctionMappingConfiguration {
  int locoAddress;
  uint8_t fnNo;
  bool fnOnOff;
  int trainLightIndex;
  TrainLightStatus trainLightStatus;
} locoFunctionMappingConfiguration[NUM_FUNCTION_MAPPINGS] =
{
  // fn1: forward mode. head light white
  {
    // head light red component on
    .locoAddress = 10020,
    .fnNo = 1,
    .fnOnOff = true,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    // head light green component on
    .locoAddress = 10020,
    .fnNo = 1,
    .fnOnOff = true,
    .trainLightIndex = 1,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    // head light blue component on
    .locoAddress = 10020,
    .fnNo = 1,
    .fnOnOff = true,
    .trainLightIndex = 2,
    .trainLightStatus = TrainLightStatus::ON
  },

  // fn2: backwards mode. head light red
  {
    // head light red component on
    .locoAddress = 10020,
    .fnNo = 2,
    .fnOnOff = true,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    // head light green component off
    .locoAddress = 10020,
    .fnNo = 2,
    .fnOnOff = true,
    .trainLightIndex = 1,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    // head light blue component off
    .locoAddress = 10020,
    .fnNo = 2,
    .fnOnOff = true,
    .trainLightIndex = 2,
    .trainLightStatus = TrainLightStatus::OFF
  },

  // fn3: head light off
  {
    // head light red component off
    .locoAddress = 10020,
    .fnNo = 3,
    .fnOnOff = true,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    // head light green component off
    .locoAddress = 10020,
    .fnNo = 3,
    .fnOnOff = true,
    .trainLightIndex = 1,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    // head light blue component off
    .locoAddress = 10020,
    .fnNo = 3,
    .fnOnOff = true,
    .trainLightIndex = 2,
    .trainLightStatus = TrainLightStatus::OFF
  },
};

// *********************************
// TRAIN LIGHT TRIGGER CONFIGURATION
// *********************************

// Triggers are used to AUTOMATICALLY switch train lights on and off

// Number of train light triggers as defined just below
#define NUM_TRAIN_LIGHT_TRIGGERS 9

// List of train light triggers
struct TrainLightTriggerConfiguration {
  int locoAddress; // set to 0 to indicate "all locos"
  LightEventType lightEventType;
  int trainLightIndex;
  TrainLightStatus trainLightStatus;
} trainLightTriggerConfiguration[NUM_TRAIN_LIGHT_TRIGGERS] =
{
  // forward mode. head light white
  {
    // head light red component on
    .locoAddress = 10020,
    .lightEventType = LightEventType::FORWARD,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    // head light green component on
    .locoAddress = 10020,
    .lightEventType = LightEventType::FORWARD,
    .trainLightIndex = 1,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    // head light blue component on
    .locoAddress = 10020,
    .lightEventType = LightEventType::FORWARD,
    .trainLightIndex = 2,
    .trainLightStatus = TrainLightStatus::ON
  },

  // backward mode. head light red
  {
    // head light red component on
    .locoAddress = 10020,
    .lightEventType = LightEventType::REVERSE,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    // head light green component off
    .locoAddress = 10020,
    .lightEventType = LightEventType::REVERSE,
    .trainLightIndex = 1,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    // head light blue component off
    .locoAddress = 10020,
    .lightEventType = LightEventType::REVERSE,
    .trainLightIndex = 2,
    .trainLightStatus = TrainLightStatus::OFF
  },

  // this section may be commented out to prevent the head and rear lights from being switched off upon stop
/*
  // stop: head light off
  {
    // head light red component off
    .locoAddress = 10020,
    .lightEventType = LightEventType::STOP,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    // head light green component off
    .locoAddress = 10020,
    .lightEventType = LightEventType::STOP,
    .trainLightIndex = 1,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    // head light blue component off
    .locoAddress = 10020,
    .lightEventType = LightEventType::STOP,
    .trainLightIndex = 2,
    .trainLightStatus = TrainLightStatus::OFF
  },
*/
};


// ************************
// CONTROLLER CONFIGURATION
// ************************

// Constants for motorshield type Lego IR Receiver 8884
#define IR_LED_PIN D5      // pin on which the IR LED is installed that controls all attached Lego IR Receiver 8884s.

// Digital output PIN to monitor controller operation (typically a LED)
bool STATUS_LED_PIN_INSTALLED = true;  // set to false if no LED is installed
uint8_t STATUS_LED_PIN = D4;
bool STATUS_LED_REVERSE = true;

// Report battery level
const bool REPORT_BATTERYLEVEL = false;           // set to true or false to allow or omit battery level reports
const int SEND_BATTERYLEVEL_INTERVAL = 60000;     // interval for sending battery level in milliseconds
const int BATTERY_PIN = A0;
const int VOLTAGE_MULTIPLIER = 20000 / 5000 - 1;  // Rbottom = 5 kOhm; Rtop = 20 kOhm; => voltage split factor
const int MAX_AI_VOLTAGE = 5100;                  // maximum analog input voltage on pin A0. Usually 5000 = 5V = 5000mV. Can be slightly adapted to correct small deviations


// ****************
// NETWORK SETTINGS
// ****************

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT true

// WiFi Hostname
// Hostnames must start with a-z, A-Z, 0-9. From 2nd character, hyphens ("-") may also be used
const char* MC_HOSTNAME = "MTC4PF-SFE1";

// Syslog application name
const char* SYSLOG_APP_NAME = "MTC4PF-SFE1";
