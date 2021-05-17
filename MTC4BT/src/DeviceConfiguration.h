#pragma once

#include "HubChannel.h"

class DeviceConfiguration
{
public:
    DeviceConfiguration(std::string address, bool isInverted, AttachedDevice attachedDevice);

    // Returns the raw device address.
    std::string GetAddress();

    // Returns the device's address interpreted as a HubChannel.
    HubChannel GetAddressAsHubChannel();

    // Returns the device's address interpreted as an ESP pin number.
    int GetAddressAsEspPinNumber();

    // Returns a boolean value indicating whether the deivce should be treated as inverted.
    bool IsInverted();

    // Returns the attached device.
    AttachedDevice GetAttachedDevice();

    // Gets the parent hub/receiver address.
    std::string GetParentAddress();

    // Sets the parent hub/receiver address.
    void SetParentAddress(std::string address);

private:
    // Can be hub/receiver channel or ESP pin number.
    std::string _address;

    // Can be hub/receiver address.
    std::string _parentAddress;

    // Boolean value indcating whether the channel should be treated as inverted.
    bool _isInverted = false;

    // Attached device.
    AttachedDevice _device = AttachedDevice::NOTHING;
};