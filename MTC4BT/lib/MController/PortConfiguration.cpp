#include <Arduino.h>

#include "PortConfiguration.h"
#include "log4MC.h"

PortConfiguration::PortConfiguration(PortType portType, std::string address, bool isInverted, AttachedDevice deviceType)
    : _portType{portType}, _address{address}, _isInverted{isInverted}, _deviceType{deviceType} {}

PortType PortConfiguration::GetPortType()
{
    return _portType;
}

std::string PortConfiguration::GetAddress()
{
    return _address;
}

int PortConfiguration::GetAddressAsEspPinNumber()
{
    if(_portType != PortType::EspPin)
    {
        log4MC::error("Trying to retrieve an ESP pin number from a non ESP pin device.");
    }

    return atoi(_address.c_str());
}

bool PortConfiguration::IsInverted()
{
    return _isInverted;
}

AttachedDevice PortConfiguration::GetAttachedDeviceType()
{
    return _deviceType;
}

std::string PortConfiguration::GetParentAddress()
{
    return _parentAddress;
}

void PortConfiguration::SetParentAddress(std::string address)
{
    _parentAddress = address;
}