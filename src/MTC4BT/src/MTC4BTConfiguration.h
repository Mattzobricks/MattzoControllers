#pragma once

#include "MCConfiguration.h"
#include "BLELocomotiveConfiguration.h"

struct MTC4BTConfiguration : MCConfiguration
{
public:
    // BLE locomotive configurations.
    std::vector<BLELocomotiveConfiguration *> Locomotives;
};