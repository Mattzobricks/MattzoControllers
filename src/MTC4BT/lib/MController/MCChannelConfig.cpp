#include <Arduino.h>

#include "MCChannelConfig.h"

MCChannelConfig::MCChannelConfig(
        MCChannel *channel, int pwrIncStep, int pwrDecStep, bool isInverted, int pwr, DeviceType deviceType
    )
    : _channel{channel}, _pwrIncStep{pwrIncStep}, _pwrDecStep{pwrDecStep}, _isInverted{isInverted}, _pwr{pwr}, _deviceType{deviceType}
    {}

MCChannel *MCChannelConfig::GetChannel()
{
    return _channel;
}

int MCChannelConfig::GetPwrIncStep()
{
    return _pwrIncStep;
}

int MCChannelConfig::GetPwrDecStep()
{
    return _pwrDecStep;
}

bool MCChannelConfig::IsInverted()
{
    return _isInverted;
}

int MCChannelConfig::GetPwr()
{
    return _pwr;
}
DeviceType MCChannelConfig::GetAttachedDeviceType()
{
    return _deviceType;
}