#include "BLEHubConfiguration.h"

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels)
{
    HubType = hubType;
    DeviceAddress = new NimBLEAddress(deviceAddress);
    Channels = channels;
}