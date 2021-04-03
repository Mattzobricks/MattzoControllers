#pragma once

#include <Arduino.h>

#include "SBrickHub.h"
#include "PUHub.h"

#define AUTO_LIGHTS_ENABLED true
#define AUTO_LIGHTS_DISABLED false

#define numHubs 4
BLEHub *hubs[numHubs];

// BLE scan duration in seconds. If the device isn't found within this timeframe the scan is aborted.
const uint32_t BLE_SCAN_DURATION_IN_SECONDS = 1;

// Duration between BLE discovery and connect attempts in seconds.
const uint32_t BLE_CONNECT_DELAY_IN_SECONDS = 2;

// Until we get this to work, we can't connect more than 3 hubs (even when we configure more below):
//  https://github.com/h2zero/NimBLE-Arduino/issues/99#issuecomment-667213152
//  The solution is actually a lot simpler. Just change the defines in the sdkconfig.h file at Arduino/hardware/espressif/esp32/tools/sdk/include/config to:
//  #define CONFIG_BTDM_CONTROLLER_BLE_MAX_CONN 9
//  #define CONFIG_BTDM_CONTROLLER_BLE_MAX_CONN_EFF 9

void configureHubs()
{
    std::vector<ChannelConfiguration> channels_YC7939;
    channels_YC7939.push_back({HubChannel::A});
    channels_YC7939.push_back({HubChannel::B, AttachedDevice::LIGHT, HubChannelDirection::FORWARD, 10, 20});
    channels_YC7939.push_back({HubChannel::C});
    channels_YC7939.push_back({HubChannel::D, AttachedDevice::MOTOR, HubChannelDirection::FORWARD, 10, 20});
    hubs[0] = new SBrickHub("YC7939", "00:07:80:d0:47:43", &channels_YC7939, 80);

    std::vector<ChannelConfiguration> channels_PT60197;
    channels_PT60197.push_back({HubChannel::A, AttachedDevice::LIGHT, HubChannelDirection::FORWARD, 5, 10});
    channels_PT60197.push_back({HubChannel::B, AttachedDevice::MOTOR, HubChannelDirection::FORWARD, 5, 10});
    hubs[1] = new PUHub("PT60197", "90:84:2b:07:13:7f", &channels_PT60197, 80);

    std::vector<ChannelConfiguration> channels_BC60052;
    channels_BC60052.push_back({HubChannel::A});
    channels_BC60052.push_back({HubChannel::B, AttachedDevice::MOTOR, HubChannelDirection::FORWARD, 10, 20});
    channels_BC60052.push_back({HubChannel::C});
    channels_BC60052.push_back({HubChannel::D, AttachedDevice::LIGHT, HubChannelDirection::FORWARD, 10, 20});
    hubs[2] = new SBrickHub("BC60052", "88:6b:0f:23:78:10", &channels_BC60052, 80);

    std::vector<ChannelConfiguration> channels_HE10233;
    channels_HE10233.push_back({HubChannel::A, AttachedDevice::LIGHT, HubChannelDirection::FORWARD, 10, 20});
    channels_HE10233.push_back({HubChannel::B});
    channels_HE10233.push_back({HubChannel::C, AttachedDevice::MOTOR, HubChannelDirection::FORWARD, 10, 20});
    channels_HE10233.push_back({HubChannel::D});
    hubs[3] = new SBrickHub("HE10233", "00:07:80:d0:3a:f2", &channels_HE10233, 80);
}