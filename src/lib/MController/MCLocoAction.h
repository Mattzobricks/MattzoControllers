#pragma once

#include <Arduino.h>

#include "MCChannel.h"

// Loco action
class MCLocoAction
{
  public:
    MCLocoAction(MCChannel *channel, int16_t targetPerc);

    MCChannel *GetChannel();

    int16_t GetTargetPowerPerc();

  private:
    // Holds the target channel.
    MCChannel *_channel;

    // Holds the target power percentage to apply to the port.
    int16_t _targetPerc;
};