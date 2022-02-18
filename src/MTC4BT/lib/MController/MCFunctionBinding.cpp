#include "MCFunctionBinding.h"

MCFunctionBinding::MCFunctionBinding(MCFunction function, MCPortConfiguration *portConfig)
    : _function{function}, _portConfig{portConfig} {}

MCFunction MCFunctionBinding::GetFunction()
{
    return _function;
}

MCPortConfiguration *MCFunctionBinding::GetPortConfiguration()
{
    return _portConfig;
}