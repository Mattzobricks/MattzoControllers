#pragma once

#include "BLELocomotiveConfiguration.h"
#include "BLERemoteConfiguration.h"
#include "MCConfiguration.h"

struct MTC4BTConfiguration : MCConfiguration {
  public:
    // BLE locomotive configurations.
    std::vector<BLELocomotiveConfiguration *> LocoConfigs;
    std::vector<BLERemoteConfiguration *> RemoteConfigs;
};