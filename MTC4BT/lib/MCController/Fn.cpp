#include "Fn.h"

Fn::Fn(MCFunction function, DeviceConfiguration *deviceConfig)
    : _function{function}, _deviceConfig{deviceConfig} {}

MCFunction Fn::GetFunction()
{
    return _function;
}

DeviceConfiguration *Fn::GetDeviceConfiguration()
{
    return _deviceConfig;
}