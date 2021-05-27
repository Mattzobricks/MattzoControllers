#pragma once

#include "enums.h"

class PortConfiguration
{
public:
    PortConfiguration(PortType portType, std::string address, bool isInverted, DeviceType deviceType);

    // Returns the type of port the device is attached to.
    PortType GetPortType();

    // Returns the raw device address.
    std::string GetAddress();

    // Returns the device's address interpreted as an ESP pin number.
    int GetAddressAsEspPinNumber();

    // Returns a boolean value indicating whether the deivce should be treated as inverted.
    bool IsInverted();

    // Returns the attached device.
    DeviceType GetAttachedDeviceType();

    // Gets the parent hub/receiver address.
    std::string GetParentAddress();

    // Sets the parent hub/receiver address.
    void SetParentAddress(std::string address);

private:
    // Type of port.
    PortType _portType;

    // Can be hub/receiver channel or ESP pin number.
    std::string _address;

    // Can be hub/receiver address.
    std::string _parentAddress;

    // Boolean value indcating whether the port should be treated as inverted.
    bool _isInverted = false;

    // Type of device attached to the port.
    DeviceType _deviceType = DeviceType::Nothing;
};