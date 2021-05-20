#include <Arduino.h>

#include "DeviceConfiguration.h"

DeviceConfiguration::DeviceConfiguration(HardwareType hwType, std::string address, bool isInverted, AttachedDevice deviceType)
    : _hwType{hwType}, _address{address}, _isInverted{isInverted}, _deviceType{deviceType} {}

HardwareType DeviceConfiguration::GetHardwareType()
{
    return _hwType;
}

std::string DeviceConfiguration::GetAddress()
{
    return _address;
}

int DeviceConfiguration::GetAddressAsEspPinNumber()
{
    return atoi(_address.c_str());
}

bool DeviceConfiguration::IsInverted()
{
    return _isInverted;
}

AttachedDevice DeviceConfiguration::GetAttachedDeviceType()
{
    return _deviceType;
}

std::string DeviceConfiguration::GetParentAddress()
{
    return _parentAddress;
}

void DeviceConfiguration::SetParentAddress(std::string address)
{
    _parentAddress = address;
}