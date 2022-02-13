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



// **********************************************************************************
// Example file for configuring the MTC4PF to control multiple LEGO IR Receivers 8884
// **********************************************************************************


// *****
// LOCOS
// *****

// Number of locos (aka. MattzoLocos) controlled by this controller
#define NUM_LOCOS 2

// List of MattzoLocos
// The parameters have the following meaning:
// - locoName: name of the loco as setup in Rocrail
// - locoAddress: address of the loco as setup in Rocrail
// - accelerationInterval: time interval for acceleration / braking (default: 100 ms)
// - accelerateStep: power increment for each acceleration step
// - brakeStep: : power decrement for each braking step
MattzoLocoConfiguration* getMattzoLocoConfiguration() {
  static MattzoLocoConfiguration locoConf[NUM_LOCOS];

  locoConf[0] = (MattzoLocoConfiguration){
    .locoName = "EME",
    .locoAddress = 10194,
    .accelerationInterval = 100,
    .accelerateStep = 20,
    .brakeStep = 20
  };
  locoConf[1] = (MattzoLocoConfiguration){
    .locoName = "MAE",
    .locoAddress = 10219,
    .accelerationInterval = 100,
    .accelerateStep = 20,
    .brakeStep = 20
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
#define NUM_MOTORSHIELDS 2

// List of motor shields that are controlled by this controller
// The parameters have the following meaning:
// - motorShieldName: usually the same as the name of the loco. If the 4DBrix WiFi Train Receiver is used, you can choose a different name here. Useful if a train has multiple 4DBrix receivers on board and the motor turning direction is different.
// - locoAddress: loco that this motor shields is attached to
// - motorShieldType: motor shield type
// - minArduinoPower: minimum power (should be 0 for LEGO IR Receiver 8884)
// - maxArduinoPower: maximum power (max. 1023)
// - configMotorA: turning direction of motor A (1 = forward, -1 = backward, 0 = unused). In case of LEGO IR Receiver 8884, this is the motor connected to the red port.
// - configMotorB: same for motor B; if IR receiver: blue port
// - irChannel: if a LEGO IR Receiver 8884 is used, the selected channel of the receiver. May be 0, 1, 2 or 3. If the loco uses multiple IR receivers on different channels, additional motor shields for the loco are required.
MattzoMotorShieldConfiguration* getMattzoMotorShieldConfiguration() {
  static MattzoMotorShieldConfiguration msConf[NUM_MOTORSHIELDS];

  msConf[0] = (MattzoMotorShieldConfiguration) {
      .motorShieldName = "EME",
      .locoAddress = 10194,
      .motorShieldType = MotorShieldType::LEGO_IR_8884,
      .minArduinoPower = MIN_ARDUINO_POWER,
      .maxArduinoPower = MAX_ARDUINO_POWER,
      .configMotorA = -1,
      .configMotorB = 0,
      .irChannel = 0
  };
  msConf[1] = (MattzoMotorShieldConfiguration) {
      .motorShieldName = "MAE",
      .locoAddress = 10219,
      .motorShieldType = MotorShieldType::LEGO_IR_8884,
      .minArduinoPower = MIN_ARDUINO_POWER,
      .maxArduinoPower = MAX_ARDUINO_POWER,
      .configMotorA = -1,
      .configMotorB = 0,
      .irChannel = 1
  };

  return msConf;
}


// *************************
// TRAIN LIGHT CONFIGURATION
// *************************

// Number of train lights controlled by this controller
#define NUM_TRAIN_LIGHTS 1

// List of train lights including their configuration
struct TrainLightConfiguration {
  // Train light type. Either some light that is directly wired to the ESP, or via an LEGO IR receiver 8884
  TrainLightType trainLightType;
  // If light is directly wired to the ESP, the output pin; else: unused
  uint8_t pin;
  // IR channel of the IR receiver that the light is connected to (relevant for LEGO IR receiver 8884 only)
  uint8_t irChannel;
  // IR port to which the PF lights are attached (MattzoPowerFunctionsPort::RED or ::BLUE; relevant for LEGO IR receiver 8884 only)
  MattzoPowerFunctionsPort irPort;
  // if directly connected: PWM value on output pin when light is off (usually 0)
  // if connected via IR: IR power value when light is off (usually 0)
  int powerLevelOff;
  // if directly connected: PWM value on output pin when light is on (max is MAX_ARDUINO_POWER = 1023)
  // if connected via IR: IR power value when light in on (max. 100 = full bright).
  int powerLevelOn;
} trainLightConfiguration[NUM_TRAIN_LIGHTS] = 
{
  {
    // head lights for Emerald Express
    .trainLightType = TrainLightType::LEGO_IR_8884,
    .pin = D0,
    .irChannel = 0,
    .irPort = MattzoPowerFunctionsPort::BLUE,
    .powerLevelOff = 0,
    .powerLevelOn = 100,
  }
};


// ******************************
// FUNCTION MAPPING CONFIGURATION
// ******************************

// Rocrail functions are used to MANUALLY switch train lights on and off

// Number of function mappings
#define NUM_FUNCTION_MAPPINGS 4

// List of function mappings
struct LocoFunctionMappingConfiguration {
  int locoAddress;
  uint8_t fnNo;
  bool fnOnOff;
  int trainLightIndex;
  TrainLightStatus trainLightStatus;
} locoFunctionMappingConfiguration[NUM_FUNCTION_MAPPINGS] =
{
  {
    .locoAddress = 10194,
    .fnNo = 1,
    .fnOnOff = true,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    .locoAddress = 10194,
    .fnNo = 2,
    .fnOnOff = true,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    .locoAddress = 10194,
    .fnNo = 3,
    .fnOnOff = true,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::FLASH
  },
  {
    .locoAddress = 10194,
    .fnNo = 4,
    .fnOnOff = true,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::BLINK
  }
};


// *********************************
// TRAIN LIGHT TRIGGER CONFIGURATION
// *********************************

// Triggers are used to AUTOMATICALLY switch train lights on and off

// Number of train light triggers as defined just below
#define NUM_TRAIN_LIGHT_TRIGGERS 5

// List of train light triggers
struct TrainLightTriggerConfiguration {
  int locoAddress; // set to 0 to indicate "all locos"
  LightEventType lightEventType;
  int trainLightIndex;
  TrainLightStatus trainLightStatus;
} trainLightTriggerConfiguration[NUM_TRAIN_LIGHT_TRIGGERS] =
{
  {
    .locoAddress = 10194,
    .lightEventType = LightEventType::STOP,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::OFF
  },
  {
    .locoAddress = 10194,
    .lightEventType = LightEventType::FORWARD,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::ON
  },
  {
    .locoAddress = 10194,
    .lightEventType = LightEventType::REVERSE,
    .trainLightIndex = 0,
    .trainLightStatus = TrainLightStatus::OFF
  },
};


// ***************************
// CONTROLLER WIRING SPECIFICS
// ***************************

// Type of motor shield directly wired to the controller.
// (The different motor shield types are defined in MTC4PF.ino)
// Set to MotorShieldType::NONE if only virtual motor shields are used!
const MotorShieldType MOTORSHIELD_TYPE = MotorShieldType::LEGO_IR_8884;

// Constants for motor shield type L298N
#define enA D0  // PWM signal pin for motor A. Relevant for L298N only.
#define enB D1  // PWM signal pin for motor B. Relevant for L298N only.

// Constants for motor shield type L298N and L9110
#define in1 D3  // pin for motor A direction control (forward).
#define in2 D4  // pin for motor A direction control (reverse).
#define in3 D5  // pin for motor B direction control (forward).
#define in4 D6  // pin for motor B direction control (reverse).

// Constants for motorshield type Lego IR Receiver 8884
#define IR_LED_PIN D5			// pin on which the IR LED is installed.

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
// NEWORK SETTINGS
// ****************

// Trigger emergency brake upon disconnect
#define TRIGGER_EBREAK_UPON_DISCONNECT true

// Syslog application name
const char* SYSLOG_APP_NAME = "MTC4PF-IR";
