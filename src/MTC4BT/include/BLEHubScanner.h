#pragma once

#include "BLEHub.h"
#include "NimBLEDevice.h"
#include <Arduino.h>

class BLEHubScanner
{
  public:
	BLEHubScanner();

	// Public members

	// Starts device discovery (if not already discovering).
	void StartDiscovery(std::vector<BLEHub *> &hubs, const uint32_t scanDurationInSeconds);

  private:
	// Private members

	// Reference to the BLE scanner used by this controller.
	NimBLEScan *_scanner;

	// Reference to the device callback.
	NimBLEAdvertisedDeviceCallbacks *_advertisedDeviceCallback;

	// Boolean value indicating whether scanner is active.
	bool _isDiscovering;
};