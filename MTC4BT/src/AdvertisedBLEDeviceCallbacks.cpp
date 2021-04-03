#include <Arduino.h>
#include "NimBLEDevice.h"
#include "BLEHub.h"
#include "AdvertisedBLEDeviceCallbacks.h"

AdvertisedBLEDeviceCallbacks::AdvertisedBLEDeviceCallbacks(BLEHub *hub) : NimBLEAdvertisedDeviceCallbacks()
{
    _hub = hub;
}

// Called for each advertising BLE server.
void AdvertisedBLEDeviceCallbacks::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
    // We have found a device, let us now see if it has the address we are looking for.
    if (advertisedDevice->getAddress().equals(*_hub->_address))
    {
        // Stop active scan.
        NimBLEDevice::getScan()->stop();

        Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Discovered our device: ");
        Serial.println(advertisedDevice->toString().c_str());

        _hub->_advertisedDevice = advertisedDevice;
        _hub->_isDiscovered = true;
    }
}