#include <Arduino.h>

#include "BLEHubScanner.h"
#include "BLEDeviceCallbacks.h"
#include "log4MC.h"

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

    log4MC::vlogf(LOG_INFO, "BLE : Scanning for %u hub(s)...", hubs.size());

    // Set the callback we want to use to be informed when we have detected a new device.
    if (_advertisedDeviceCallback == nullptr)
    {
        _advertisedDeviceCallback = new BLEDeviceCallbacks(hubs);
    }

    scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
    scanner->start(scanDurationInSeconds, false);

    log4MC::vlogf(LOG_INFO, "BLE : Scanning for %u hub(s) aborted.", hubs.size());

    // Discovery stopped.
    _isDiscovering = false;
}