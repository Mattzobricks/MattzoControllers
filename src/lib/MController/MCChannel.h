#pragma once

#include "enums.h"

// Controller channel
class MCChannel
{
public:
    MCChannel(ChannelType portType, std::string address);

    // Returns the type of channel the device is attached to.
    ChannelType GetChannelType();

    // Returns the raw channel address.
    std::string GetAddress();

    // Returns the channel's address interpreted as an ESP pin number.
    int GetAddressAsEspPinNumber();

    // Gets the parent hub/receiver address.
    std::string GetParentAddress();

    // Sets the parent hub/receiver address.
    void SetParentAddress(const std::string address);

private:
    // Type of channel.
    ChannelType _portType;

    // Can be hub/receiver channel or ESP pin number.
    std::string _address;

    // Can be hub/receiver address.
    std::string _parentAddress;
};