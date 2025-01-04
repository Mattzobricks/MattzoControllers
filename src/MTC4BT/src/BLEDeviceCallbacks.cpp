#include "BLEDeviceCallbacks.h"
#include "BLEHub.h"
#include "NimBLEDevice.h"
#include "log4MC.h"
#include <Arduino.h>

BLEDeviceCallbacks::BLEDeviceCallbacks(std::vector<BLEHub *> hubs) : NimBLEAdvertisedDeviceCallbacks()
{
	_hubs = hubs;
}

// Called for each advertising BLE server.
void BLEDeviceCallbacks::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
	// We have found a device, let's see if it has an address we are looking for.
	for (BLEHub *hub : _hubs) {
		if (advertisedDevice->getAddress().equals(*hub->_config->DeviceAddress)) {
			log4MC::vlogf(LOG_INFO, "BLE : Discovered hub: %s (%s).", advertisedDevice->getName().c_str(), advertisedDevice->getAddress().toString().c_str());

			hub->_advertisedDevice = advertisedDevice;
			hub->_isDiscovered = true;

			return;
		}
	}

	log4MC::vlogf(LOG_INFO, "BLE : Discovered unknown device: %s (%s).", advertisedDevice->getName().c_str(), advertisedDevice->getAddress().toString().c_str());
	if (_hubs.size() != 0) {
		// ignore found device when not in scan mode!
		NimBLEDevice::addIgnored(advertisedDevice->getAddress());
	}
}