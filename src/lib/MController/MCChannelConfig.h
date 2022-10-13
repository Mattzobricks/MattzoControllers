#pragma once

#include "MCChannel.h"
#include "enums.h"

// Controller channel config
class MCChannelConfig
{
  public:
    MCChannelConfig(MCChannel *channel, int pwrIncStep, int pwrDecStep, bool isInverted, DeviceType deviceType);

    // Returns the channel.
    MCChannel *GetChannel();

    // Returns a boolean value indicating whether the attached device' polarity is inverted.
    bool IsInverted();

    // Returns the power step used when increasing power on the channel.
    int GetPwrIncStep();

    // Returns the power step used when decreasing power on the channel.
    int GetPwrDecStep();

    // Returns the type of device attached to the channel.
    DeviceType GetAttachedDeviceType();

  private:
    // Type of port.
    MCChannel *_channel;

    // Power step to use when increasing power on the channel.
    int _pwrIncStep;

    // Power step to use when decreasing power on the channel.
    int _pwrDecStep;

    // Boolean value indicating whether the attached device' polarity is inverted.
    bool _isInverted;

    // Type of device attached to the channel.
    DeviceType _deviceType = DeviceType::Nothing;
};