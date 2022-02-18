#include <Arduino.h>
#include <ArduinoJson.h>

#include "BLELocomotiveConfiguration.h"

BLELocomotiveConfiguration::BLELocomotiveConfiguration(uint address, std::string name, std::vector<BLEHubConfiguration *> hubs, std::vector<MCFunctionBinding *> functions, int16_t speedStep, int16_t brakeStep, int16_t lightPerc, bool autoLightsEnabled, bool enabled)
{
    _address = address;
    _name = name;
    _hubs = hubs;
    _functions = functions;
    _speedStep = speedStep;
    _brakeStep = brakeStep;
    _lightPerc = lightPerc;
    _autoLightsEnabled = autoLightsEnabled;
    _enabled = enabled;
}