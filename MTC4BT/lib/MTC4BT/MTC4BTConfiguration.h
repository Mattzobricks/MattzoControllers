#pragma once

#include "BLELocomotive.h"

struct MTC4BTConfiguration
{
public:
    // BLE locomotives.
    std::vector<BLELocomotive *> Locomotives;
};