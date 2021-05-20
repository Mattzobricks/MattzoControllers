#pragma once

#include "MCConfiguration.h"
#include "DeviceConfiguration.h"
#include "BLELocomotive.h"
#include "Fn.h"

struct MTC4BTConfiguration : MCConfiguration
{
public:
    // BLE locomotives.
    std::vector<BLELocomotive *> Locomotives;
};