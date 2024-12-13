#pragma once

#include "BLEHubConfiguration.h"
// #include "MCLocoEvent.h"

class BLERemoteConfiguration
{
  public:
    BLERemoteConfiguration(std::vector<BLEHubConfiguration *> hubs);//, std::vector<MCLocoEvent *> events);

    std::string _name;
    std::vector<BLEHubConfiguration *> _hubs;
    //std::vector<MCLocoEvent *> _events;
};