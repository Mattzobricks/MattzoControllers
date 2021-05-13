#pragma once

#include "BLELocomotive.h"

struct MTC4BTConfiguration
{
public:
    // Controller name.
    const char *ControllerName;

    // BLE locomotives.
    std::vector<BLELocomotive *> Locomotives;
};