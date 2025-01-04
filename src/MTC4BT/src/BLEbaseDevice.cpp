/*
 * BLEbaseDevice
 */

#include "BLEbaseDevice.h"
#include "MController.h"
#include "log4MC.h"

BLEbaseDevice::BLEbaseDevice(MController *controller)
	: _controller{controller}
{
}

bool BLEbaseDevice::AllHubsConnected()
{
	return true;
}

void BLEbaseDevice::BlinkLights(int durationInMs)
{
	if (!AllHubsConnected()) {
		// Ignore blink request.
		// log4MC::vlogf(LOG_INFO, "Loco: %s ignored blink lights request because not all its hubs are connected (yet).", _config->_name.c_str());
		return;
	}

	// Handle blink on lights attached to channels of our hubs.
	for (BLEHub *hub : Hubs) {
		hub->BlinkLights(durationInMs);
	}
}

void BLEbaseDevice::SetHubLedColor(HubLedColor color)
{
	if (!AllHubsConnected()) {
		// Ignore led color request.
		// log4MC::vlogf(LOG_INFO, "Loco: %s ignored led color request because not all its hubs are connected (yet).", _config->_name.c_str());
		return;
	}

	// Set hub's onboard led color.
	for (BLEHub *hub : Hubs) {
		hub->SetHubLedColor(color);
	}
}

uint BLEbaseDevice::GetHubCount()
{
	return Hubs.size();
}

BLEHub *BLEbaseDevice::GetHub(uint index)
{
	return Hubs.at(index);
}

BLEHub *BLEbaseDevice::getHubByAddress(std::string address)
{
	for (BLEHub *hub : Hubs) {
		if (hub->GetRawAddress().compare(address) == 0) {
			return hub;
		}
	}

	return nullptr;
}