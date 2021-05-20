#pragma once

#include "DeviceConfiguration.h"
#include "enums.h"

// Function definition.
class Fn
{
public:
    Fn(MCFunction function, DeviceConfiguration *deviceConfig);

    MCFunction GetFunction();
    DeviceConfiguration *GetDeviceConfiguration();

private:
    // Function.
    MCFunction _function;

    // Attached device config.
    DeviceConfiguration *_deviceConfig;
};