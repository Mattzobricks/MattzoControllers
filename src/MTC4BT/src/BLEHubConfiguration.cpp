#include "BLEHubConfiguration.h"

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, uint8_t powerlevel)
{
	HubType = hubType;
	DeviceAddress = new NimBLEAddress(deviceAddress, BLE_ADDR_PUBLIC);
	Channels = channels;
	powerLevel = powerlevel;
	mode = noMode;
}

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, PUbuttonByType *newButtons, std::vector<freeListItem *> newFreeListItems)
{
	HubType = hubType;
	DeviceAddress = new NimBLEAddress(deviceAddress, BLE_ADDR_PUBLIC);
	Channels = channels;
	mode = listMode;
	list.buttons = newButtons;
	list.freeListItems = newFreeListItems;
}

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<MCChannelConfig *> channels, PUbuttonList *freeButtons, HubLedColor newColor)
{
	HubType = hubType;
	DeviceAddress = new NimBLEAddress(deviceAddress, BLE_ADDR_PUBLIC);
	Channels = channels;
	mode = freeMode;
	buttons = freeButtons;
	remoteColor = newColor;
}
