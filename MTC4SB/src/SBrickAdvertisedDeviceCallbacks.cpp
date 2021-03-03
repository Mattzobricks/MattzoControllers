#include <Arduino.h>
#include "NimBLEDevice.h"
#include "SBrickHubClient.h"
#include "SBrickAdvertisedDeviceCallbacks.h"

SBrickAdvertisedDeviceCallbacks::SBrickAdvertisedDeviceCallbacks(SBrickHubClient *sbrick) : NimBLEAdvertisedDeviceCallbacks()
{
  _psbrick = sbrick;
}

// Called for each advertising BLE server.
void SBrickAdvertisedDeviceCallbacks::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
  // We have found a device, let us now see if it has the address we are looking for.
  if (advertisedDevice->getAddress().equals(*_psbrick->_address))
  {
    // Stop active scan.
    NimBLEDevice::getScan()->stop();

    Serial.print("Discovered our device: ");
    Serial.println(advertisedDevice->toString().c_str());
    _psbrick->_isDiscovered = true;
  }
}