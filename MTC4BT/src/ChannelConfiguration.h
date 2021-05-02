#pragma once

#include "HubChannel.h"

struct ChannelConfiguration
{
public:
    ChannelConfiguration(HubChannel c)
    {
        channel = c;
    }

    ChannelConfiguration(HubChannel c, AttachedDevice d)
    {
        channel = c;
        device = d;
    }

    ChannelConfiguration(HubChannel c, AttachedDevice d, int16_t s, int16_t b)
    {
        channel = c;
        device = d;
        speedStep = s;
        brakeStep = b;
    }

    ChannelConfiguration(HubChannel c, AttachedDevice d, int16_t s, int16_t b, HubChannelDirection dir)
    {
        channel = c;
        device = d;
        speedStep = s;
        brakeStep = b;
        direction = dir;
    }

    // Channel of the Hub.
    HubChannel channel;

    // Device attached to the channel.
    AttachedDevice device = AttachedDevice::NOTHING;

    // Channel power direction.
    HubChannelDirection direction = HubChannelDirection::FORWARD;

    // Raw speed step to use when increasing speed.
    int16_t speedStep = 10;

    // Raw speed step to use when decreasing speed.
    int16_t brakeStep = 20;
};