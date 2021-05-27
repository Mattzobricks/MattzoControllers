#pragma once

#include "PortConfiguration.h"
#include "enums.h"

// Function definition.
class Fn
{
public:
    Fn(MCFunction function, PortConfiguration *portConfig);

    // Returns the controller function assigned to this function definition.
    MCFunction GetFunction();

    // Returns a reference to the configuration of the port assigned to this function definition.
    PortConfiguration *GetPortConfiguration();

private:
    // Controller function.
    MCFunction _function;

    // Reference to the configuration of the port assigned to this function definition
    PortConfiguration *_portConfig;
};