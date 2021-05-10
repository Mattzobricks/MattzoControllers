#include <Arduino.h>
#include "NimBLEDevice.h"
#include "BLEHub.h"
#include "AdvertisedBLEDeviceCallbacks.h"
#include "log4MC.h"

AdvertisedBLEDeviceCallbacks::AdvertisedBLEDeviceCallbacks(std::vector<BLEHub *> hubs) : NimBLEAdvertisedDeviceCallbacks()
{
    _hubs = hubs;
}

// Called for each advertising BLE server.
void AdvertisedBLEDeviceCallbacks::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
    bool advertisedDeviceFound = false;

    // We have found a device, let's see if it has an address we are looking for.
    for (int i = 0; i < _hubs.size(); i++)
    {
        BLEHub *hub = _hubs.at(i);

        if (advertisedDevice->getAddress().equals(*hub->_config->DeviceAddress))
        {
            log4MC::vlogf(LOG_INFO, "BLE : Discovered hub %s (%s).", advertisedDevice->getName().c_str(), advertisedDevice->getAddress().toString().c_str());

            hub->_advertisedDevice = advertisedDevice;
            hub->_isDiscovered = true;
            advertisedDeviceFound = true;
        }
    }

    if (!advertisedDeviceFound)
    {
        log4MC::vlogf(LOG_INFO, "BLE : Discovered unknown device %s (%s).", advertisedDevice->getName().c_str(), advertisedDevice->getAddress().toString().c_str());
    }

    // If we have no more hubs to discover, we can stop scanning now.
    bool moreHubsToDiscover = false;
    for (int i = 0; i < _hubs.size(); i++)
    {
        BLEHub *hub = _hubs.at(i);
        if (!hub->IsDiscovered())
        {
            moreHubsToDiscover = true;
            break;
        }
    }

    if (!moreHubsToDiscover)
    {
        // Stop active scan.
        NimBLEDevice::getScan()->stop();
    }
}