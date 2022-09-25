#pragma once

#include <Arduino.h>

#include "NimBLEAddress.h"
#include "MCChannelConfig.h"
#include <vector>

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
    BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, bool enabled = true);

    // Type of Hub.
    BLEHubType HubType;

    // MAC address of the Hub.
    NimBLEAddress *DeviceAddress;

    // Hub channels.
    std::vector<MCChannelConfig *> Channels;

    // Boolean value indicating whether this Hub is in use.
    bool Enabled;
};