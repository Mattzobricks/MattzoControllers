#include <Arduino.h>

#include "BLELocomotiveConfiguration.h"

BLELocomotiveConfiguration::BLELocomotiveConfiguration(uint address, std::string name, std::vector<BLEHubConfiguration *> hubs, int16_t lightPerc, bool autoLightsEnabled, bool enabled)
{
    _address = address;
    _name = name;
    _hubs = hubs;
    _lightPerc = lightPerc;
    _autoLightsEnabled = autoLightsEnabled;
    _enabled = enabled;
}