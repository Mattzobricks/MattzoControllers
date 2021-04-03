#pragma once

#include "NimBLEDevice.h"
#include "BLEHub.h"

class AdvertisedBLEDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
public:
    AdvertisedBLEDeviceCallbacks(BLEHub *hub);

private:
    void onResult(NimBLEAdvertisedDevice *advertisedDevice);

    BLEHub *_hub;
};