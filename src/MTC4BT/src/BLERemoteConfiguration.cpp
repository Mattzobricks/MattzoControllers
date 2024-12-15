#include <Arduino.h>
#include <ArduinoJson.h>

#include "BLERemoteConfiguration.h"

BLERemoteConfiguration::BLERemoteConfiguration(std::vector<BLEHubConfiguration *> hubs)
    : _hubs{hubs}
{
}