#pragma once

#include <Arduino.h>

#include "BLELocomotive.h"
#include "SBrickHub.h"
#include "PUHub.h"

#define AUTO_LIGHTS_ON_ENABLED true
#define HUB_DISABLED false

// WARNING: The controller can manage up to 9 hubs in total. This is a BLE hardware limitation.
// Always make sure the total number of hubs in all your locos together doesn't exceed this number!

// Always make sure the `numLocos` you specify here, matches the number of locos you actually configure below.
#define numLocos 1
BLELocomotive *locos[numLocos];

// BLE scan duration in seconds. If the device isn't found within this timeframe the scan is aborted.
const uint32_t BLE_SCAN_DURATION_IN_SECONDS = 1;

// Duration between BLE discovery and connect attempts in seconds.
const uint32_t BLE_CONNECT_DELAY_IN_SECONDS = 3;

/*
    TODO: !!! PLEASE NOTE: DOCS BELOW ARE OUTDATED !!!

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
*/

void configureLocos()
{
    std::vector<BLEHubConfiguration *> hubs_YC7939;
    std::vector<ChannelConfiguration *> hub0_YC7939_channels;
    hub0_YC7939_channels.push_back(new ChannelConfiguration(HubChannel::B, AttachedDevice::LIGHT));
    hub0_YC7939_channels.push_back(new ChannelConfiguration(HubChannel::D, AttachedDevice::MOTOR));
    BLEHubConfiguration *hub0_YC7939 = new BLEHubConfiguration(BLEHubType::SBrick, "00:07:80:d0:47:43", &hub0_YC7939_channels);
    hubs_YC7939.push_back(hub0_YC7939);
    locos[0] = new BLELocomotive(new BLELocomotiveConfiguration(1, "YC7939", hubs_YC7939));
}