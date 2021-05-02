#pragma once

#include <Arduino.h>

#include "NimBLEAddress.h"
#include "ChannelConfiguration.h"

enum BLEHubType
{
    // Powered Up Hub (Lego).
    PU,

    // SBrick (Vengit).
    SBrick
};

class BLEHubConfiguration
{
public:
    BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<ChannelConfiguration *> *channels, int16_t lightPerc = 100, bool autoLightsOnEnabled = false, bool enabled = true);

    // Type of Hub.
    BLEHubType HubType;

    // MAC address of the Hub.
    NimBLEAddress *DeviceAddress;

    // Hub channels.
    std::vector<ChannelConfiguration *> *Channels;

    // Percentage of power to use when turning lights on.
    int16_t LightPerc;

    // Boolean value indicating whether lights should be on while driving.
    bool AutoLightsEnabled;

    // Boolean value indicating whether this Hub is in use.
    bool Enabled;
};