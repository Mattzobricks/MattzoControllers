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
    BuWizz2,

    // Powered Up remote
    PUController

};

// String switch paridgam
struct bleHubTypeMap : public std::map<std::string, BLEHubType> {
    bleHubTypeMap()
    {
        this->operator[]("PU") = BLEHubType::PU;
        this->operator[]("SBrick") = BLEHubType::SBrick;
        this->operator[]("BuWizz2") = BLEHubType::BuWizz2;
        this->operator[]("PUController") = BLEHubType::PUController;
    };
    ~bleHubTypeMap() {}
};

// Map BuWizz2 powerlevel names to numers
struct buwizzPowerMap : public std::map<std::string, uint8_t> {
    buwizzPowerMap()
    {
        this->operator[]("disabled") = 0;
        this->operator[]("slow") = 1;
        this->operator[]("normal") = 2;
        this->operator[]("fast") = 3;
        this->operator[]("ldcrs") = 4;
    };
    ~buwizzPowerMap() {}
};

class BLEHubConfiguration
{
  public:
    BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels,uint8_t powerlevel );

    // Type of Hub.
    BLEHubType HubType;

    // MAC address of the Hub.
    NimBLEAddress *DeviceAddress;

    // Hub channels.
    std::vector<MCChannelConfig *> Channels;

    //hub power level, only valid for BuWizz2, the rest is ignored
    uint8_t powerLevel;
};