#pragma once

#include "DeviceConfiguration.h"
#include "MTC4BTFunction.h"

// Function definition.
class Fn
{
public:
    Fn(MTC4BTFunction function, DeviceConfiguration *device);

    MTC4BTFunction GetFunction();
    DeviceConfiguration *GetDevice();

private:
    MTC4BTFunction _function;
    DeviceConfiguration *_device;
};