#pragma once

#include "BLELocomotiveConfiguration.h"
#include "MCConfiguration.h"

struct MTC4BTConfiguration : MCConfiguration {
  public:
    // BLE locomotive configurations.
    std::vector<BLELocomotiveConfiguration *> Locomotives;
};