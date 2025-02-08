#pragma once

#include "BLEHub.h"
#include "NimBLEDevice.h"

class BLEDeviceCallbacks : public NimBLEScanCallbacks
{
  public:
	BLEDeviceCallbacks(std::vector<BLEHub *> hubs);

  private:
	void onResult(const NimBLEAdvertisedDevice *advertisedDevice);

	std::vector<BLEHub *> _hubs;
};