#include <Arduino.h>

#include "MCChannelConfig.h"

MCChannelConfig::MCChannelConfig(MCChannel *channel, int pwrIncStep, int pwrDecStep, bool isInverted, DeviceType deviceType)
    : _channel{channel}, _pwrIncStep{pwrIncStep}, _pwrDecStep{pwrDecStep}, _isInverted{isInverted}, _deviceType{deviceType} {}

MCChannel *MCChannelConfig::GetChannel()
{
    return _channel;
}

bool MCChannelConfig::IsInverted()
{
    return _isInverted;
}

int MCChannelConfig::GetPwrIncStep()
{
    return _pwrIncStep;
}

int MCChannelConfig::GetPwrDecStep()
{
    return _pwrDecStep;
}

DeviceType MCChannelConfig::GetAttachedDeviceType()
{
    return _deviceType;
}