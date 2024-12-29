#include "BLEHubConfiguration.h"

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, uint8_t powerlevel)
{
    HubType = hubType;
    DeviceAddress = new NimBLEAddress(deviceAddress);
    Channels = channels;
    powerLevel = powerlevel;
    mode = noMode;
}

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, PUbuttonByType *newButtons, std::vector<freeListItem *> newFreeListItems)
{
    HubType = hubType;
    DeviceAddress = new NimBLEAddress(deviceAddress);
    Channels = channels;
    mode = listMode;
    list.buttons = newButtons;
    list.freeListItems = newFreeListItems;
}

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, PUbuttonList * freeButtons)
{
    HubType = hubType;
    DeviceAddress = new NimBLEAddress(deviceAddress);
    Channels = channels;
    mode = freeMode;
    buttons = freeButtons;
}
