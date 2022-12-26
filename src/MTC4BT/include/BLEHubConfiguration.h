#pragma once

#include <Arduino.h>

#include "MCChannelConfig.h"
#include "NimBLEAddress.h"
#include <vector>

enum BLEHubType {
    // Powered Up Hub (Lego).
    PU,

    // SBrick (Vengit).
    SBrick,

    // ADDNEWDEVICE * comment for places where to add code for a new device
    // Dummy to test memory leak
    DUMMY,
};

// String switch paridgam
struct bleHubTypeMap : public std::map<std::string, BLEHubType> {
    bleHubTypeMap()
    {   
        // ADDNEWDEVICE * comment for places where to add code for a new device
        this->operator[]("DUMMY") = BLEHubType::DUMMY;
        
        this->operator[]("PU") = BLEHubType::PU;
        this->operator[]("SBrick") = BLEHubType::SBrick;
    };
    ~bleHubTypeMap() {}
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