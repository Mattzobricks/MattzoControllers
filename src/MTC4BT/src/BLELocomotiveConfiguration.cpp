#include <Arduino.h>
#include <ArduinoJson.h>

#include "BLELocomotiveConfiguration.h"

BLELocomotiveConfiguration::BLELocomotiveConfiguration(uint address, std::string name, std::vector<BLEHubConfiguration *> hubs,
                                                       std::vector<MCLocoEvent *> events, bool enabled)
    : _address{address}, _name{name}, _hubs{hubs}, _events{events}, _enabled{enabled} {}