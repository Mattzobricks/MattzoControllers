#include <Arduino.h>

#include "MCPortConfiguration.h"
#include "log4MC.h"

MCPortConfiguration::MCPortConfiguration(PortType portType, std::string address, bool isInverted, DeviceType deviceType)
    : _portType{portType}, _address{address}, _isInverted{isInverted}, _deviceType{deviceType} {}

PortType MCPortConfiguration::GetPortType()
{
    return _portType;
}

std::string MCPortConfiguration::GetAddress()
{
    return _address;
}

int MCPortConfiguration::GetAddressAsEspPinNumber()
{
    if(_portType != PortType::EspPin)
    {
        log4MC::error("Trying to retrieve an ESP pin number from a non ESP pin device.");
    }

    return atoi(_address.c_str());
}

bool MCPortConfiguration::IsInverted()
{
    return _isInverted;
}

DeviceType MCPortConfiguration::GetAttachedDeviceType()
{
    return _deviceType;
}

std::string MCPortConfiguration::GetParentAddress()
{
    return _parentAddress;
}

void MCPortConfiguration::SetParentAddress(std::string address)
{
    _parentAddress = address;
}