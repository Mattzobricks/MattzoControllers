#pragma once

#include "ChannelController.h"

struct ChannelConfiguration
{
    // Channel of the Hub.
    HubChannel channel;

    // Device attached to the channel.
    AttachedDevice device;

    // Channel power direction.
    HubChannelDirection direction;

    // Raw speed step to use when increasing speed.
    int16_t speedStep;

    // Raw speed step to use when decreasing speed.
    int16_t brakeStep;
};