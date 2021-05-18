#include "Fn.h"

Fn::Fn(MTC4BTFunction function, DeviceConfiguration *deviceConfig)
    : _function{function}, _deviceConfig{deviceConfig} {}

MTC4BTFunction Fn::GetFunction()
{
    return _function;
}

DeviceConfiguration *Fn::GetDeviceConfiguration()
{
    return _deviceConfig;
}