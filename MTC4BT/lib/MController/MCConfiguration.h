#pragma once

#include <Arduino.h>

#include "MCPortConfiguration.h"
#include "MCFunctionBinding.h"

struct MCConfiguration
{
public:
    // Controller name.
    const char *ControllerName;

    // ESP32 pins.
    std::vector<MCPortConfiguration *> EspPins;

    // Functions.
    std::vector<MCFunctionBinding *> Functions;
};