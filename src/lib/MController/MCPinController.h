#pragma once

#include <Arduino.h>

#include "MCChannelConfig.h"
#include "MCChannelController.h"

class MCPinController : public MCChannelController
{
  public:
    MCPinController(MCChannelConfig *config);

    // Returns the channel's address interpreted as an ESP pin number.
    int GetEspPinNumber();
};