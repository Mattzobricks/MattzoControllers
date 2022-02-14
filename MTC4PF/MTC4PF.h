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


// ***********************
// ENUMS, STRUCTS, CLASSES
// ***********************

// MotorShieldType represents the type of the motor shield that is attached to the controller
// L298N and L9110 are motorshields that are physically connected to the controller
// LEGO_IR_8884 is connected via an infrared LED that must be wired to the controller
// WIFI_TRAIN_RECEIVER_4DBRIX 4 is connected via WiFi
enum struct MotorShieldType
{
  NONE,
  L298N,
  L9110,
  LEGO_IR_8884,
  WIFI_TRAIN_RECEIVER_4DBRIX
};

// Min and max useful power for Arduino based motor shields
const int MIN_ARDUINO_POWER = 400;   // default minimum useful arduino power. May be overwritten in motor shield configuration
const int MAX_ARDUINO_POWER = 1023;  // default maximum arduino power. May be overwritten in motor shield configuration

// Train light types
enum struct TrainLightType
{
  ESP_OUTPUT_PIN,  // light is wired to a pin of the ESP-8266
  POWER_FUNCTIONS  // light is wired to a power functions motor port (e.g. a Power Functions light)
};

// Status of a train light
enum struct TrainLightStatus
{
  OFF = 0x0,
  ON = 0x1,
  FLASH = 0x2,
  BLINK = 0x3
};

// List of events that may trigger train light actions
enum struct LightEventType
{
  STOP = 0x0,
  FORWARD = 0x1,
  REVERSE = 0x2
};

// Loco configuration structure
struct MattzoLocoConfiguration {
  String locoName;
  int locoAddress;
  int accelerationInterval;
  int accelerateStep;
  int brakeStep;
};

// Motorshield configuration structure
struct MattzoMotorShieldConfiguration {
  int locoAddress;
  MotorShieldType motorShieldType;
  uint8_t L298N_enA;
  uint8_t L298N_enB;
  uint8_t in1;
  uint8_t in2;
  uint8_t in3;
  uint8_t in4;
  int minArduinoPower;
  int maxArduinoPower;
  int configMotorA;
  int configMotorB;
  int irChannel;
};

// This class contains a locomotive object as defined in Rocrail
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
};

// This class contains a motor shield
class MattzoMotorShield {
public:
  // Members
  int _locoAddress;   // address of the Rocrail loco in which this motor shield is built-in
  MotorShieldType _motorShieldType;
  uint8_t _L298N_enA;
  uint8_t _L298N_enB;
  uint8_t _in1;
  uint8_t _in2;
  uint8_t _in3;
  uint8_t _in4;
  int _minArduinoPower = MIN_ARDUINO_POWER;  // minimum useful power setting
  int _maxArduinoPower = MAX_ARDUINO_POWER;  // maximum useful power setting
  int _configMotorA = 0;  // 1 = forward, 0 not installed, -1 = reverse
  int _configMotorB = 0;
  int _irChannel = -1;    // IR channel. May be 0, 1, 2 or 3. -1 = not installed
  MattzoPowerFunctionsPwm pfPowerLevelRed;
  MattzoPowerFunctionsPwm pfPowerLevelBlue;

  // Methods
  void initMattzoMotorShield(MattzoMotorShieldConfiguration c) {
    _locoAddress = c.locoAddress;
    _motorShieldType = c.motorShieldType;
    _L298N_enA = c.L298N_enA;
    _L298N_enB = c.L298N_enB;
    _in1 = c.in1;
    _in2 = c.in2;
    _in3 = c.in3;
    _in4 = c.in4;
    _minArduinoPower = c.minArduinoPower;
    _maxArduinoPower = c.maxArduinoPower;
    _configMotorA = c.configMotorA;
    _configMotorB = c.configMotorB;
    _irChannel = c.irChannel;
  }

  bool checkLocoAddress(int locoAddress) {
    return (locoAddress == 0 || locoAddress == _locoAddress);
  }
};

// This struct represents a train light and its status
struct TrainLight {
  TrainLightStatus desiredTrainLightState = TrainLightStatus::OFF;
  int actualPowerLevel = 0;
};


// *********
// CONSTANTS
// *********

// maximum speed to power functions speed mapping function
#define MAX_IR_POWERVALUE 100

// Waiting time between two consecutive infrared commands
#define WAIT_BETWEEN_IR_TRANSMISSIONS_MS 1000
