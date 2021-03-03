#pragma once

#include "NimBLEDevice.h"
#include "SBrickHubClient.h"

class SBrickAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
public:
    SBrickAdvertisedDeviceCallbacks(SBrickHubClient* sbrick);

private:
    void onResult(NimBLEAdvertisedDevice* advertisedDevice);

    SBrickHubClient* _psbrick;
};