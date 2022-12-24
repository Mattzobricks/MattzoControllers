#pragma once

#include "BLEHubConfiguration.h"
#include "MCLocoEvent.h"

class BLELocomotiveConfiguration
{
  public:
    BLELocomotiveConfiguration(uint address, std::string name, std::vector<BLEHubConfiguration *> hubs, std::vector<MCLocoEvent *> events, bool enabled = true);

    uint _address;
    std::string _name;
    std::vector<BLEHubConfiguration *> _hubs;
    std::vector<MCLocoEvent *> _events;
    bool _enabled;
};