#pragma once

#include <Arduino.h>

#include "DeviceConfiguration.h"
#include "Fn.h"

struct MCConfiguration
{
public:
    // Controller name.
    const char *ControllerName;

    // ESP32 pins.
    std::vector<DeviceConfiguration *> EspPins;

    // Functions.
    std::vector<Fn *> Functions;
};