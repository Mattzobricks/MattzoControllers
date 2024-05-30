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
// Example file for configuring the MTC4PF to control a loco with lights but no motor
// **********************************************************************************

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
MattzoLocoConfiguration *getMattzoLocoConfiguration()
{
    static MattzoLocoConfiguration locoConf[NUM_LOCOS];

    locoConf[0] = (MattzoLocoConfiguration){
        .locoName = "SFE",
        .locoAddress = 10020,
        .accelerationInterval = 100,
        .accelerateStep = 2,
        .brakeStep = 10};

    return locoConf;
}

// *************
// MOTOR SHIELDS
// *************
// Number of motor shields connected to this controller
const int NUM_MOTORSHIELDS = 0;
MattzoMotorShieldConfiguration *getMattzoMotorShieldConfiguration()
{
    static MattzoMotorShieldConfiguration msConf[NUM_MOTORSHIELDS];
    return msConf;
}

// *************************
// TRAIN LIGHT CONFIGURATION
// *************************

// Number of train lights controlled by this controller
#define NUM_TRAIN_LIGHTS 3

// List of train lights including their configuration
TTrainLightConfiguration trainLightConfiguration[NUM_TRAIN_LIGHTS] =
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
TLocoFunctionMappingConfiguration locoFunctionMappingConfiguration[NUM_FUNCTION_MAPPINGS] =
{
    // fn1: forward mode. head light red
    {
        // head light red component on
        .locoAddress = 10020,
        .fnNo = 1,
        .fnOnOff = true,
        .trainLightIndex = 0,
        .trainLightStatus = TrainLightStatus::ON},
    {
        // head light green component off
        .locoAddress = 10020,
        .fnNo = 1,
        .fnOnOff = true,
        .trainLightIndex = 1,
        .trainLightStatus = TrainLightStatus::OFF},
    {
        // head light blue component off
        .locoAddress = 10020,
        .fnNo = 1,
        .fnOnOff = true,
        .trainLightIndex = 2,
        .trainLightStatus = TrainLightStatus::OFF},

    // fn2: backwards mode. head light white
    {
        // head light red component on
        .locoAddress = 10020,
        .fnNo = 2,
        .fnOnOff = true,
        .trainLightIndex = 0,
        .trainLightStatus = TrainLightStatus::ON},
    {
        // head light green component on
        .locoAddress = 10020,
        .fnNo = 2,
        .fnOnOff = true,
        .trainLightIndex = 1,
        .trainLightStatus = TrainLightStatus::ON},
    {
        // head light blue component on
        .locoAddress = 10020,
        .fnNo = 2,
        .fnOnOff = true,
        .trainLightIndex = 2,
        .trainLightStatus = TrainLightStatus::ON},

    // fn3: head light off
    {
        // head light red component off
        .locoAddress = 10020,
        .fnNo = 3,
        .fnOnOff = true,
        .trainLightIndex = 0,
        .trainLightStatus = TrainLightStatus::OFF},
    {
        // head light green component off
        .locoAddress = 10020,
        .fnNo = 3,
        .fnOnOff = true,
        .trainLightIndex = 1,
        .trainLightStatus = TrainLightStatus::OFF},
    {
        // head light blue component off
        .locoAddress = 10020,
        .fnNo = 3,
        .fnOnOff = true,
        .trainLightIndex = 2,
        .trainLightStatus = TrainLightStatus::OFF},
};

// *********************************
// TRAIN LIGHT TRIGGER CONFIGURATION
// *********************************

// Triggers are used to AUTOMATICALLY switch train lights on and off

// Number of train light triggers as defined just below
#define NUM_TRAIN_LIGHT_TRIGGERS 9

// List of train light triggers
TTrainLightTriggerConfiguration trainLightTriggerConfiguration[NUM_TRAIN_LIGHT_TRIGGERS] =
{
    // forward mode. head light red
    {
        // head light red component on
        .locoAddress = 10020,
        .lightEventType = LightEventType::FORWARD,
        .trainLightIndex = 0,
        .trainLightStatus = TrainLightStatus::ON},
    {
        // head light green component off
        .locoAddress = 10020,
        .lightEventType = LightEventType::FORWARD,
        .trainLightIndex = 1,
        .trainLightStatus = TrainLightStatus::OFF},
    {
        // head light blue component off
        .locoAddress = 10020,
        .lightEventType = LightEventType::FORWARD,
        .trainLightIndex = 2,
        .trainLightStatus = TrainLightStatus::OFF},

    // backward mode. head light white
    {
        // head light red component on
        .locoAddress = 10020,
        .lightEventType = LightEventType::REVERSE,
        .trainLightIndex = 0,
        .trainLightStatus = TrainLightStatus::ON},
    {
        // head light green component on
        .locoAddress = 10020,
        .lightEventType = LightEventType::REVERSE,
        .trainLightIndex = 1,
        .trainLightStatus = TrainLightStatus::ON},
    {
        // head light blue component on
        .locoAddress = 10020,
        .lightEventType = LightEventType::REVERSE,
        .trainLightIndex = 2,
        .trainLightStatus = TrainLightStatus::ON},

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
const u_int8_t IR_LED_PIN = D5; // pin on which the IR LED is installed that controls all attached Lego IR Receiver 8884s.

// Digital output pin to monitor controller operation (typically a LED)
// Set to false if no status LED is installed
const bool STATUS_LED_PIN_INSTALLED = true;
// If installed, the pin controlling the status LED
const uint8_t STATUS_LED_PIN = D4;
// If installed, set to true to flip high/low state of the status led pin
const bool STATUS_LED_REVERSE = true;
// Power level of the status LED (0..1023)
// Recommended max. power levels: white: 800, blue: 600, green: 500, yellow: 350, red: 300
const int STATUS_LED_POWER = 300;

// Report battery level
const bool REPORT_BATTERYLEVEL = false;       // set to true or false to allow or omit battery level reports
const int SEND_BATTERYLEVEL_INTERVAL = 60000; // interval for sending battery level in milliseconds
const int BATTERY_PIN = A0;
const int VOLTAGE_MULTIPLIER = 20000 / 5000 - 1; // Rbottom = 5 kOhm; Rtop = 20 kOhm; => voltage split factor
const int MAX_AI_VOLTAGE = 5100;                 // maximum analog input voltage on pin A0. Usually 5000 = 5V = 5000mV. Can be slightly adapted to correct small deviations

// ****************
// NETWORK SETTINGS
// ****************

// Trigger emergency brake upon disconnect
const bool TRIGGER_EBREAK_UPON_DISCONNECT = false;

// WiFi Hostname
// Hostnames must start with a-z, A-Z, 0-9. From 2nd character, hyphens ("-") may also be used
const char *MC_HOSTNAME = "MTC4PF-SFE2";

// Syslog application name
const char *SYSLOG_APP_NAME = "MTC4PF-SFE2";
