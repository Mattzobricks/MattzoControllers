#pragma once

#include "NimBLEDevice.h"
#include "BLEHub.h"

class BLEDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
public:
    BLEDeviceCallbacks(std::vector<BLEHub *> hubs);

private:
    void onResult(NimBLEAdvertisedDevice *advertisedDevice);

    std::vector<BLEHub *> _hubs;
};