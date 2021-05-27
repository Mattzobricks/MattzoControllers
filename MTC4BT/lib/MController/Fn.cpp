#include "Fn.h"

Fn::Fn(MCFunction function, PortConfiguration *portConfig)
    : _function{function}, _portConfig{portConfig} {}

MCFunction Fn::GetFunction()
{
    return _function;
}

PortConfiguration *Fn::GetPortConfiguration()
{
    return _portConfig;
}