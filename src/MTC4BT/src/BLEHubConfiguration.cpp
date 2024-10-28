#include "BLEHubConfiguration.h"

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, uint8_t powerlevel)
{
    HubType = hubType;
    DeviceAddress = new NimBLEAddress(deviceAddress);
    Channels = channels;
    powerLevel = powerlevel;
}