#pragma once

#include <Arduino.h>

#include "SBrickHub.h"
#include "PUHub.h"

#define AUTO_LIGHTS_ON_ENABLED true
#define HUB_DISABLED false

// You can configure up to 9 hubs.
// Always make sure the `numHubs` you specify here, matches the number of hubs you actually configure below.
#define numHubs 2
BLEHub *hubs[numHubs];

// BLE scan duration in seconds. If the device isn't found within this timeframe the scan is aborted.
const uint32_t BLE_SCAN_DURATION_IN_SECONDS = 1;

// Duration between BLE discovery and connect attempts in seconds.
const uint32_t BLE_CONNECT_DELAY_IN_SECONDS = 3;

/*
    SBrick/PU Hub constructor parameters (sequencial):
    
    deviceName:             This should match the @ID param of your loco in Rocrail.
    deviceAddress:          This should match the BLE address of your SBrick or PU Hub.
    channels:               This channel configuration should mimic the real world channel usage (more on ChannelConfiguration parameters below):
                            - SBricks have four (unmarked) channels (A, B, C and D).
                            - PU Hubs have two (marked) channels (A and B).
                            - You only have to define channels that have a device attached to it (that you want to control).
    lightPerc:              This configures the power level for the light channels as a percentage (%) of full power:
                            - Default value: 100.
                            - Valid values: 0 (off) - 100 (full power).
    autoLightsOnEnabled:    You can set AUTO_LIGHTS_ON_ENABLED, if you want the light channel(s) to turn on automatically whenever the loco is driving and off when it stops:
                            - Default value: false.
    enabled:                You can set HUB_DISABLED, if you want this hub to be disabled for now:
                            - Default value: true.

    ChannelConfiguration parameters (sequencial):

    channel:                This should match the channel on the SBrick or PU Hub:
                            - Valid values for PU Hub: HubChannel::A, HubChannel::B.
                            - Valid values for SBrick: HubChannel::A, HubChannel::B, HubChannel::C, HubChannel::D.
    device:                 This should match the type of device attached to the channel:
                            - Valid values: AttachedDevice::NOTHING (0), AttachedDevice::MOTOR (1), AttachedDevice::LIGHT (2).
                            - Default value: AttachedDevice::NOTHING.
    speedStep:              This parameter can be used to configure the step used to increase the speed with when accerating:
                            - Default value: 10.
                            - Valid values: 1 - 254.
                            - Value recommended: 10-25 (lower means a smoother but slower increase of speed).
    brakeStep:              This parameter can be used to configure the step used to decrease the speed with when decelerating:
                            - Default value: 20.
                            - Valid values: 1 - 254.
                            - Value recommended: 10-25 (higher means a quicker stop).
    direction:              This should match the direction in which the motor is attached to the loco:
                            - Default value: HubChannelDirection::FORWARD.
                            - Valid values: HubChannelDirection::FORWARD (0), HubChannelDirection::REVERSE (1)
                            - PLEASE NOTE THIS PARAMETER IS CURRENTLY NOT USED YET!
*/

void configureHubs()
{
    std::vector<ChannelConfiguration> channels_YC7939;
    channels_YC7939.push_back({HubChannel::B, AttachedDevice::LIGHT});
    channels_YC7939.push_back({HubChannel::D, AttachedDevice::MOTOR, 10, 20, HubChannelDirection::REVERSE});
    hubs[0] = new SBrickHub("YC7939", "00:07:80:d0:47:43", &channels_YC7939, 80);

    std::vector<ChannelConfiguration> channels_PT60197;
    channels_PT60197.push_back({HubChannel::A, AttachedDevice::LIGHT, 5, 10});
    channels_PT60197.push_back({HubChannel::B, AttachedDevice::MOTOR, 5, 10});
    hubs[1] = new PUHub("PT60197", "90:84:2b:07:13:7f", &channels_PT60197, 80);
}