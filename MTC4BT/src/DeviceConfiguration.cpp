#include <Arduino.h>

#include "DeviceConfiguration.h"

DeviceConfiguration::DeviceConfiguration(std::string address, bool isInverted, AttachedDevice attachedDevice)
    : _address{address}, _isInverted{isInverted}, _device{attachedDevice} {}

std::string DeviceConfiguration::GetAddress()
{
    return _address;
}

HubChannel DeviceConfiguration::GetAddressAsHubChannel()
{
    return hubChannelMap()[_address];
}

int DeviceConfiguration::GetAddressAsEspPinNumber()
{
    return atoi(_address.c_str());
}

bool DeviceConfiguration::IsInverted()
{
    return _isInverted;
}

AttachedDevice DeviceConfiguration::GetAttachedDevice()
{
    return _device;
}

std::string DeviceConfiguration::GetParentAddress()
{
    return _parentAddress;
}

void DeviceConfiguration::SetParentAddress(std::string address)
{
    _parentAddress = address;
}