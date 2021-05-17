#pragma once

#include "DeviceConfiguration.h"
#include "BLELocomotive.h"
#include "Fn.h"

struct MTC4BTConfiguration
{
public:
    // Controller name.
    const char *ControllerName;

    // ESP32 pins.
    std::vector<DeviceConfiguration *> EspPins;

    // Functions.
    std::vector<Fn *> Functions;

    // BLE locomotives.
    std::vector<BLELocomotive *> Locomotives;
};