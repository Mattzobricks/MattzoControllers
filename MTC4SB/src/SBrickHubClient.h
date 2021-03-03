#pragma once

#include <Arduino.h>
#include "NimBLEDevice.h"

class SBrickHubClient
{
public:
  SBrickHubClient(std::string deviceName, std::string deviceAddress);
  void StartDiscovery(NimBLEScan *scanner, const uint32_t scanDurationInSeconds = 3);
  bool Connect();
  bool IsDiscovered();
  bool IsConnected();
  std::string getDeviceName();

private:
  bool connectToServer();
  // static void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
  bool attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID);

  std::string _deviceName;
  NimBLEAddress* _address;
  NimBLEClient* _sbrick;
  NimBLEAdvertisedDeviceCallbacks* _advertisedDeviceCallback;
  NimBLEClientCallbacks* _clientCallback;
  bool _isDiscovering;
  bool _isDiscovered;
  bool _isConnected;
  NimBLERemoteService* _remoteControlService;
  NimBLERemoteCharacteristic* _remoteControlCharacteristic;
  NimBLERemoteCharacteristic* _genericAccessCharacteristic;
  NimBLERemoteCharacteristic* _deviceInformationCharacteristic;

  // SBrickClientCallback and SBrickAdvertisedDeviceCallbacks can access private members of SBrickHubClient.
  friend class SBrickClientCallback;
  friend class SBrickAdvertisedDeviceCallbacks;
};