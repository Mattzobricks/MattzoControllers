#include "Fn.h"

Fn::Fn(MTC4BTFunction function, DeviceConfiguration *device)
    : _function{function}, _device{device} {}

MTC4BTFunction Fn::GetFunction()
{
    return _function;
}

DeviceConfiguration *Fn::GetDevice()
{
    return _device;
}