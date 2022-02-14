// MattzoSwitchController Firmware
// Author: Dr. Matthias Runte
// Copyright 2020, 2021 by Dr. Matthias Runte
// License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// Power management for PCA9685
// The PWM signals on the PCA9685 can be automatically turned off after a servo operation to prevent servos from overheat and to save electricity.
// Time after which servos will go to sleep mode (in milliseconds; 3000 = 3 sec.)
// MUST BE GREATER THAN SERVO_DETACH_DELAY BY AT LEAST 20 ms (one PWM cycle on 50 Hz)!
#define PCA9685_POWER_OFF_AFTER_MS 2000

// SERVO CONSTANTS AND STRUCTS
// Delay after which servo is detached after flipping a switch (for directly connected servos only)
#define SERVO_DETACH_DELAY 1000
// Maximum time that the detach procedure procedure will wait until the PWM signal is low and therefore ready to be detached (for directly connected servos only)
#define MAX_WAIT_FOR_LOW_MS 100

struct MattzoServo {
  Servo servo;      // Servo object to control servos
  boolean isAttached = false;
  unsigned long lastSwitchingAction_ms = 0;
};

// SWITCH CONSTANTS
// Default values for TrixBrix switches (in case servo angles are not transmitted)
#define SWITCHSERVO_MIN_ALLOWED 40   // minimum accepted servo angle from Rocrail. Anything below this value is treated as misconfiguration and is neglected and reset to SWITCHSERVO_MIN.
#define SWITCHSERVO_MIN 75           // a good first guess for the minimum angle of TrixBrix servos is 70
#define SWITCHSERVO_MAX 85           // a good first guess for the maximum angle of TrixBrix servos is 90
#define SWITCHSERVO_MAX_ALLOWED 120  // maximum accepted servo angle from Rocrail. Anything above this value is treated as misconfiguration and is neglected and reset to SWITCHSERVO_MAX.

// SENSOR CONSTANTS
// Time in milliseconds until release event is reported after sensor has lost contact
#define SENSOR_RELEASE_TICKS_MS 100

// LEVEL CROSSING STRUCTS
enum struct LevelCrossingStatus
{
  OPEN = 0x1,
  CLOSED = 0x2,
};

struct LevelCrossing {
  LevelCrossingStatus levelCrossingStatus = LevelCrossingStatus::OPEN;
  unsigned long lastStatusChangeTime_ms = 0;

  bool boomBarrierActionInProgress = true;
  bool closeBoomsImmediately = false;
  float servoAnglePrimaryBooms = levelCrossingConfiguration.bbAnglePrimaryUp;
  float servoAngleSecondaryBooms = levelCrossingConfiguration.bbAngleSecondaryUp;
  float servoAngleIncrementPerSec = 0;
  float servoTargetAnglePrimaryBooms = levelCrossingConfiguration.bbAnglePrimaryUp;
  float servoTargetAngleSecondaryBooms = levelCrossingConfiguration.bbAngleSecondaryUp;
  unsigned long lastBoomBarrierTick_ms = 0;

  unsigned int sensorEventCounter[LC_NUM_TRACKS][2][2];
  bool trackOccupied[LC_NUM_TRACKS];
  unsigned long trackOccupiedTimeout_ms[LC_NUM_TRACKS];
};

// BASCULE BRIDGE STRUCTS
enum struct BridgeStatus
{
  CLOSED,
  OPENING,
  OPENED,
  CLOSING,
  UNDEFINED,
  ERRoR,
  UNCHANGED
};

enum struct BridgeLeafStatus
{
  CLOSED,
  OPENING0,
  OPENING1,
  OPENING2,
  OPENING3,
  OPENED,
  CLOSING0,
  CLOSING1,
  CLOSING2,
  CLOSING3,
  UNDEFINED,
  ERRoR,
  UNCHANGED
};

enum struct BridgeCommand
{
  UP = 0,
  DOWN = 1,
  NONE = 2
};

struct BridgeLeaf {
  Servo bridgeLeafServo;
  BridgeLeafStatus leafStatus = BridgeLeafStatus::UNDEFINED;
  unsigned long leafTimer;
};

struct Bridge {
  BridgeStatus bridgeStatus = BridgeStatus::UNDEFINED;
  BridgeCommand bridgeCommand = BridgeCommand::NONE;
  BridgeLeaf bridgeLeaf[NUM_BASCULE_BRIDGE_LEAFS];
};


// SPEEDOMETER STRUCTS
// Maximum number of magnets that can be attached to a train
#define SM_MAX_VALUES 20

struct Speedometer {
  bool occupied = false;

  int startSensor;
  int endSensor;
  int wheelcounter[2] = { -1, -1 };

  float startTime[SM_MAX_VALUES];
  float endTime[SM_MAX_VALUES];
  float trainSpeed[SM_MAX_VALUES];
  float trainLength[SM_MAX_VALUES];

  unsigned long lastMeasurementEvent = -99999;
  unsigned long measurementDone = -99999;
  unsigned long animationDelay = -99999;

  float actualTrainSpeed = 0;
  float actualTrainLength = 0;
};
