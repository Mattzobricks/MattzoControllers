#pragma once

#include <Arduino.h>

#include "BLEHubChannel.h"
#include "MCChannelConfig.h"
#include "MCChannelController.h"

class BLEHubChannelController : public MCChannelController
{
  public:
    BLEHubChannelController(MCChannelConfig *config);

    // Returns the controlled hub channel.
    BLEHubChannel GetHubChannel();
};