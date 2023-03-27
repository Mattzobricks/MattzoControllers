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
MattzoLocoConfiguration *getMattzoLocoConfiguration()
{
    static MattzoLocoConfiguration locoConf[NUM_LOCOS];

    locoConf[0] = (MattzoLocoConfiguration){
        .locoName = "TGV2",
        .locoAddress = 10233,
        .accelerationInterval = 100,
        .accelerateStep = 2,
        .brakeStep = 10};

    return locoConf;
}

// *************
// MOTOR SHIELDS
// *************
// Number of motor shields connected to this controller
const int NUM_MOTORSHIELDS = 1;

// List of motor shields that are controlled by this controller
// The parameters have the following meaning:
// - locoAddress: loco that this motor shields is attached to
// - motorShieldType: motor shield type
// - L298N_enA, L298N_enB: PWM signal pin for motor A / B, if L298N is used.
// - in1..in4: pin for motor direction control for motor shields L298N and L9110 (in1: forward motor A, in2: reverse motor A, in3: forward motor B, in4: reverse motor B).
// - minArduinoPower: minimum power setting for Arduino based motor shields. You might need to adapt this to your specific shield and motor. 200 might be a good value for a start. Should be 0 for LEGO IR Receiver 8884.
// - maxArduinoPower: maximum power setting for Arduino based motor shields (max. 1023). You might need to adapt this to your specific shield and motor. 400 might be a good value for a start.
// - configMotorA: turning direction of motor A (1 = forward, -1 = backward, 0 = unused). In case of LEGO IR Receiver 8884, this is the motor connected to the red port.
// - configMotorB: same for motor B; if IR receiver: blue port
// - irChannel: if a LEGO IR Receiver 8884 is used, the selected channel of the receiver. May be 0, 1, 2 or 3. If the loco uses multiple IR receivers on different channels, additional motor shields for the loco are required.
MattzoMotorShieldConfiguration *getMattzoMotorShieldConfiguration()
{
    static MattzoMotorShieldConfiguration msConf[NUM_MOTORSHIELDS];

    msConf[0] = (MattzoMotorShieldConfiguration){
        .locoAddress = 10233,
        .motorShieldType = MotorShieldType::L9110,
        .L298N_enA = 0,
        .L298N_enB = 0,
        .in1 = D3,
        .in2 = D4,
        .in3 = D5,
        .in4 = D6,
        .minArduinoPower = MIN_ARDUINO_POWER,
        .maxArduinoPower = MAX_ARDUINO_POWER,
        .configMotorA = 1,
        .configMotorB = 0,
        .irChannel = -1};

    return msConf;
}

// *************************
// TRAIN LIGHT CONFIGURATION
// *************************

// Number of train lights controlled by this controller
#define NUM_TRAIN_LIGHTS 2

// List of train lights including their configuration
TTrainLightConfiguration trainLightConfiguration[NUM_TRAIN_LIGHTS] =
    {
        {
            // 0: head light / white
            .trainLightType = TrainLightType::ESP_OUTPUT_PIN,
            .pin = D0,
            .motorShieldIndex = 0,
            .motorPortIndex = -1,
            .powerLevelOff = 0,
            .powerLevelOn = MAX_ARDUINO_POWER,
        },
        {
            // 1: head light / red
            .trainLightType = TrainLightType::ESP_OUTPUT_PIN,
            .pin = D2,
            .motorShieldIndex = 0,
            .motorPortIndex = -1,
            .powerLevelOff = 0,
            .powerLevelOn = 850,
        },
};

// ******************************
// FUNCTION MAPPING CONFIGURATION
// ******************************

// Rocrail functions are used to MANUALLY switch train lights on and off

// Number of function mappings
#define NUM_FUNCTION_MAPPINGS 6

// List of function mappings
TLocoFunctionMappingConfiguration locoFunctionMappingConfiguration[NUM_FUNCTION_MAPPINGS] =
    {
        // fn1: forward mode. head lights red
        {
            // head lights white off
            .locoAddress = 10233,
            .fnNo = 1,
            .fnOnOff = true,
            .trainLightIndex = 0,
            .trainLightStatus = TrainLightStatus::OFF},
        {// head lights red on
         .locoAddress = 10233,
         .fnNo = 1,
         .fnOnOff = true,
         .trainLightIndex = 1,
         .trainLightStatus = TrainLightStatus::ON},

        // fn2: backwards mode. head lights white
        {
            // head lights white on
            .locoAddress = 10233,
            .fnNo = 2,
            .fnOnOff = true,
            .trainLightIndex = 0,
            .trainLightStatus = TrainLightStatus::ON},
        {// head lights red off
         .locoAddress = 10233,
         .fnNo = 2,
         .fnOnOff = true,
         .trainLightIndex = 1,
         .trainLightStatus = TrainLightStatus::OFF},

        // fn3: head lights off
        {
            // head lights white off
            .locoAddress = 10233,
            .fnNo = 3,
            .fnOnOff = true,
            .trainLightIndex = 0,
            .trainLightStatus = TrainLightStatus::OFF},
        {// head lights red off
         .locoAddress = 10233,
         .fnNo = 3,
         .fnOnOff = true,
         .trainLightIndex = 1,
         .trainLightStatus = TrainLightStatus::OFF},
};

// *********************************
// TRAIN LIGHT TRIGGER CONFIGURATION
// *********************************

// Triggers are used to AUTOMATICALLY switch train lights on and off

// Number of train light triggers as defined just below
#define NUM_TRAIN_LIGHT_TRIGGERS 6

// List of train light triggers
TTrainLightTriggerConfiguration trainLightTriggerConfiguration[NUM_TRAIN_LIGHT_TRIGGERS] =
    {
        // forward mode. head lights red
        {
            // head lights white off
            .locoAddress = 10233,
            .lightEventType = LightEventType::FORWARD,
            .trainLightIndex = 0,
            .trainLightStatus = TrainLightStatus::OFF},
        {// head lights red on
         .locoAddress = 10233,
         .lightEventType = LightEventType::FORWARD,
         .trainLightIndex = 1,
         .trainLightStatus = TrainLightStatus::ON},

        // backward mode. head lights white
        {
            // head lights white on
            .locoAddress = 10233,
            .lightEventType = LightEventType::REVERSE,
            .trainLightIndex = 0,
            .trainLightStatus = TrainLightStatus::ON},
        {// head lights red off
         .locoAddress = 10233,
         .lightEventType = LightEventType::REVERSE,
         .trainLightIndex = 1,
         .trainLightStatus = TrainLightStatus::OFF},

        // this section may be commented out to prevent the head and rear lights from being switched off upon stop
        // stop: head lights off
        {
            // head lights white off
            .locoAddress = 10233,
            .lightEventType = LightEventType::STOP,
            .trainLightIndex = 0,
            .trainLightStatus = TrainLightStatus::OFF},
        {// head lights red off
         .locoAddress = 10233,
         .lightEventType = LightEventType::STOP,
         .trainLightIndex = 1,
         .trainLightStatus = TrainLightStatus::OFF},
};

// ************************
// CONTROLLER CONFIGURATION
// ************************

// Constants for motorshield type Lego IR Receiver 8884
const u_int8_t IR_LED_PIN = D7; // pin on which the IR LED is installed that controls all attached Lego IR Receiver 8884s.

// Digital output pin to monitor controller operation (typically a LED)
// Set to false if no status LED is installed
const bool STATUS_LED_PIN_INSTALLED = true;
// If installed, the pin controlling the status LED
const uint8_t STATUS_LED_PIN = D8;
// If installed, set to true to flip high/low state of the status led pin
const bool STATUS_LED_REVERSE = false;
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
const bool TRIGGER_EBREAK_UPON_DISCONNECT = true;

// WiFi Hostname
// Hostnames must start with a-z, A-Z, 0-9. From 2nd character, hyphens ("-") may also be used
const char *MC_HOSTNAME = "MTC4PF-TGV2";

// Syslog application name
const char *SYSLOG_APP_NAME = "MTC4PF-TGV2";
