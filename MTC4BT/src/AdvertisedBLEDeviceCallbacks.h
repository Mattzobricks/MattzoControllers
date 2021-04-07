#pragma once

#include "NimBLEDevice.h"
#include "BLEHub.h"

class AdvertisedBLEDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
public:
    AdvertisedBLEDeviceCallbacks(std::vector<BLEHub *> hubs);

private:
    void onResult(NimBLEAdvertisedDevice *advertisedDevice);

    std::vector<BLEHub *> _hubs;
};