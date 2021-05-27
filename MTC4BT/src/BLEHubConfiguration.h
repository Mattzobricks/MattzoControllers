#pragma once

#include <Arduino.h>

#include "NimBLEAddress.h"
#include "PortConfiguration.h"

enum BLEHubType
{
    // Powered Up Hub (Lego).
    PU,

    // SBrick (Vengit).
    SBrick
};

// String switch paridgam   
struct bleHubTypeMap : public std::map<std::string, BLEHubType>
{
    bleHubTypeMap()
    {
        this->operator[]("PU") = BLEHubType::PU;
        this->operator[]("SBrick") = BLEHubType::SBrick;
    };
    ~bleHubTypeMap(){}
};

class BLEHubConfiguration
{
public:
    BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<PortConfiguration *> channels, int16_t lightPerc = 100, bool autoLightsOnEnabled = false, bool enabled = true);

    // Type of Hub.
    BLEHubType HubType;

    // MAC address of the Hub.
    NimBLEAddress *DeviceAddress;

    // Hub channels.
    std::vector<PortConfiguration *> Channels;

    // Percentage of power to use when turning lights on.
    int16_t LightPerc;

    // Boolean value indicating whether lights should be on while driving.
    bool AutoLightsEnabled;

    // Boolean value indicating whether this Hub is in use.
    bool Enabled;
};