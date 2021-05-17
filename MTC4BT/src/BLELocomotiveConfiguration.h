#pragma once

#include "BLEHubConfiguration.h"
#include "Fn.h"

class BLELocomotiveConfiguration
{
public:
    BLELocomotiveConfiguration(uint address, std::string name, std::vector<BLEHubConfiguration *> hubs, std::vector<Fn *> functions, int16_t speedStep = 10, int16_t brakeStep = 20, int16_t lightPerc = 100, bool autoLightsEnabled = false, bool enabled = true);

    uint _address;
    std::string _name;
    std::vector<BLEHubConfiguration *> _hubs;
    std::vector<Fn *> _functions;
    int16_t _speedStep;
    int16_t _brakeStep;
    int16_t _lightPerc;
    bool _autoLightsEnabled;
    bool _enabled;
};