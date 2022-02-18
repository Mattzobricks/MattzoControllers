#pragma once

#include <Arduino.h>
#include "NimBLEDevice.h"
#include "BLEHub.h"

class BLEHubScanner
{
public:
    BLEHubScanner();

    // Public members

    // Starts device discovery (if not already discovering).
    void StartDiscovery(NimBLEScan *scanner, std::vector<BLEHub*>& hubs, const uint32_t scanDurationInSeconds);

    // Private members
    NimBLEAdvertisedDeviceCallbacks *_advertisedDeviceCallback;
    bool _isDiscovering;
};