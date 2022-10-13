#pragma once

#include "BLEHub.h"
#include "NimBLEDevice.h"

class BLEDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
  public:
    BLEDeviceCallbacks(std::vector<BLEHub *> hubs);

  private:
    void onResult(NimBLEAdvertisedDevice *advertisedDevice);

    std::vector<BLEHub *> _hubs;
};