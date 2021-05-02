#pragma once

#include <Arduino.h>

#include "BLEHubConfiguration.h"

class BLELocomotiveConfiguration
{
public:
    BLELocomotiveConfiguration(uint address, std::string name, std::vector<BLEHubConfiguration *> hubs, int16_t lightPerc = 100, bool autoLightsEnabled = false, bool enabled = true);

    uint _address;
    std::string _name;
    std::vector<BLEHubConfiguration *> _hubs;
    int16_t _lightPerc;
    bool _autoLightsEnabled;
    bool _enabled;
};