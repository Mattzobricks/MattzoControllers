#include <Arduino.h>
#include "NimBLEDevice.h"
#include "BLEHub.h"
#include "BLEDeviceCallbacks.h"
#include "log4MC.h"

BLEDeviceCallbacks::BLEDeviceCallbacks(std::vector<BLEHub *> hubs) : NimBLEAdvertisedDeviceCallbacks()
{
    _hubs = hubs;
}

// Called for each advertising BLE server.
void BLEDeviceCallbacks::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
    bool advertisedDeviceFound = false;

    // We have found a device, let's see if it has an address we are looking for.
    for (BLEHub *hub : _hubs)
    {
        if (advertisedDevice->getAddress().equals(*hub->_config->DeviceAddress))
        {
            log4MC::vlogf(LOG_INFO, "BLE : Discovered hub: %s (%s).", advertisedDevice->getName().c_str(), advertisedDevice->getAddress().toString().c_str());

            hub->_advertisedDevice = advertisedDevice;
            hub->_isDiscovered = true;
            advertisedDeviceFound = true;
        }
    }

    if (!advertisedDeviceFound)
    {
        log4MC::vlogf(LOG_INFO, "BLE : Discovered unknown device: %s (%s).", advertisedDevice->getName().c_str(), advertisedDevice->getAddress().toString().c_str());
    }
}