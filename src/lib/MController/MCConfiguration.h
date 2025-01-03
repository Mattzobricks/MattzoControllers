#pragma once

#include "MCChannelConfig.h"
#include <Arduino.h>
#include <vector>

struct MCConfiguration
{
public:
    // Controller name.
    char *ControllerName;

    // ESP32 pins.
    std::vector<MCChannelConfig *> EspPins;
};