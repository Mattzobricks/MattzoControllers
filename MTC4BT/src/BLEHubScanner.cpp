#include <Arduino.h>

#include "NimBLEDevice.h"

#include "BLEHubScanner.h"
#include "AdvertisedBLEDeviceCallbacks.h"

BLEHubScanner::BLEHubScanner()
{
    _advertisedDeviceCallback = nullptr;
    _isDiscovering = false;
}

void BLEHubScanner::StartDiscovery(NimBLEScan *scanner, std::vector<BLEHub *> &hubs, const uint32_t scanDurationInSeconds)
{
    if (_isDiscovering)
    {
        return;
    }

    // Start discovery.
    _isDiscovering = true;

    Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Scanning for " + hubs.size() + " hub(s)...");
    // Set the callback we want to use to be informed when we have detected a new device.
    if (_advertisedDeviceCallback == nullptr)
    {
        _advertisedDeviceCallback = new AdvertisedBLEDeviceCallbacks(hubs);
    }
    scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
    scanner->start(scanDurationInSeconds, false);
    Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Scanning for " + hubs.size() + " hub(s) aborted.");

    _isDiscovering = false;
}