#pragma once

#include <Arduino.h>

#include "PortConfiguration.h"
#include "Fn.h"

struct MCConfiguration
{
public:
    // Controller name.
    const char *ControllerName;

    // ESP32 pins.
    std::vector<PortConfiguration *> EspPins;

    // Functions.
    std::vector<Fn *> Functions;
};