#pragma once

#include "HubChannel.h"
#include "enums.h"

class DeviceConfiguration
{
public:
    DeviceConfiguration(HardwareType hwType, std::string address, bool isInverted, AttachedDevice deviceType);

    // Returns the type of hardware the device is attached to.
    HardwareType GetHardwareType();

    // Returns the raw device address.
    std::string GetAddress();

    // Returns the device's address interpreted as a HubChannel.
    HubChannel GetAddressAsHubChannel();

    // Returns the device's address interpreted as an ESP pin number.
    int GetAddressAsEspPinNumber();

    // Returns a boolean value indicating whether the deivce should be treated as inverted.
    bool IsInverted();

    // Returns the attached device.
    AttachedDevice GetAttachedDeviceType();

    // Gets the parent hub/receiver address.
    std::string GetParentAddress();

    // Sets the parent hub/receiver address.
    void SetParentAddress(std::string address);

private:
    // Type of hardware.
    HardwareType _hwType;

    // Can be hub/receiver channel or ESP pin number.
    std::string _address;

    // Can be hub/receiver address.
    std::string _parentAddress;

    // Boolean value indcating whether the channel should be treated as inverted.
    bool _isInverted = false;

    // Attached device type.
    AttachedDevice _deviceType = AttachedDevice::NOTHING;
};