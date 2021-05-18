#pragma once

#include "DeviceConfiguration.h"
#include "MTC4BTFunction.h"

// Function definition.
class Fn
{
public:
    Fn(MTC4BTFunction function, DeviceConfiguration *deviceConfig);

    MTC4BTFunction GetFunction();
    DeviceConfiguration *GetDeviceConfiguration();

private:
    // Function.
    MTC4BTFunction _function;

    // Attached device config.
    DeviceConfiguration *_deviceConfig;
};