#pragma once

#include <Arduino.h>

#include "MCChannelConfig.h"

struct MCConfiguration
{
public:
    // Controller name.
    const char *ControllerName;

    // ESP32 pins.
    std::vector<MCChannelConfig *> EspPins;
};