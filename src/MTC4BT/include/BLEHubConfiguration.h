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

    // BuWizz2
    BuWizz2

};

// String switch paridgam
struct bleHubTypeMap : public std::map<std::string, BLEHubType> {
    bleHubTypeMap()
    {
        this->operator[]("PU") = BLEHubType::PU;
        this->operator[]("SBrick") = BLEHubType::SBrick;
        this->operator[]("BuWizz2") = BLEHubType::BuWizz2;
    };
    ~bleHubTypeMap() {}
};

class BLEHubConfiguration
{
  public:
    BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels);

    // Type of Hub.
    BLEHubType HubType;

    // MAC address of the Hub.
    NimBLEAddress *DeviceAddress;

    // Hub channels.
    std::vector<MCChannelConfig *> Channels;
};