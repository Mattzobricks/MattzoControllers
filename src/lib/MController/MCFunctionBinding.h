#pragma once

#include "MCPortConfiguration.h"
#include "enums.h"

// Function binding.
class MCFunctionBinding
{
public:
    MCFunctionBinding(MCFunction function, MCPortConfiguration *portConfig);

    // Returns the controller function assigned to this function binding.
    MCFunction GetFunction();

    // Returns a reference to the configuration of the port assigned to this function binding.
    MCPortConfiguration *GetPortConfiguration();

private:
    // Controller function.
    MCFunction _function;

    // Reference to the configuration of the port assigned to this function binding.
    MCPortConfiguration *_portConfig;
};