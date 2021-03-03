#include <Arduino.h>

#include "SBrickHubClient.h"
#include "NimBLEDevice.h"
#include "SBrickConst.h"
#include "SBrickClientCallback.h"
#include "SBrickAdvertisedDeviceCallbacks.h"

/*
*   - setWatchdogTimeout()
*   - subscribeToNotifications(callback)
*       pMyRemoteCharacteristic->registerForNotify(notifyCallback)
*       void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t length, bool isNotify)
* SBrickHub.Drive(command, command, command, command)
* SBrickHub.DriveChannel(channel, command)
* SBrickHub.EmergencyBreak()
*/

static BLEUUID sbrickRemoteControlServiceUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
static BLEUUID sbrickRemoteControlCharacteristicUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);

// Public members

SBrickHubClient::SBrickHubClient(std::string deviceName, std::string deviceAddress)
{
  _deviceName = deviceName;
  _address = new NimBLEAddress(deviceAddress);
  _sbrick = nullptr;
  _advertisedDeviceCallback = nullptr;
  _clientCallback = nullptr;
  _isDiscovering = false;
  _isDiscovered = false;
  _isConnected = false;
  _remoteControlService = nullptr;
  _remoteControlCharacteristic = nullptr;
  _genericAccessCharacteristic = nullptr;
  _deviceInformationCharacteristic = nullptr;
}

/// <summary>
/// Start SBrick Hub discovery.
/// </summary>
void SBrickHubClient::StartDiscovery(NimBLEScan *scanner, const uint32_t scanDurationInSeconds)
{
  if (_isDiscovering)
  {
    return;
  }

  // Start discovery.
  _isDiscovering = true;
  _isDiscovered = false;

  Serial.println("[" + String(xPortGetCoreID()) + "] SBrick: Scan started");
  // Set the callback we want to use to be informed when we have detected a new device.
  if (_advertisedDeviceCallback == nullptr)
  {
    _advertisedDeviceCallback = new SBrickAdvertisedDeviceCallbacks(this);
  }
  scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
  scanner->start(scanDurationInSeconds, false);
  Serial.println("[" + String(xPortGetCoreID()) + "] SBrick: Scan stopped");

  _isDiscovering = false;
}

/// <summary>
/// Connect to the SBrick Hub.
/// </summary>
bool SBrickHubClient::Connect()
{
  return connectToServer();
}

bool SBrickHubClient::IsDiscovered()
{
  return _isDiscovered;
}

bool SBrickHubClient::IsConnected()
{
  return _isConnected;
}

std::string SBrickHubClient::getDeviceName()
{
  return _deviceName;
}

/// <summary>
/// Sets the watchdog timeout (0D &lt; timeout in 0.1 secs, 1 byte &gt;)
/// The purpose of the watchdog is to stop driving in case of an application failure.
/// Watchdog starts when the first DRIVE command is issued during a connection.
/// Watchdog is stopped when all channels are either set to zero drive, or are braking.
/// The value is saved to the persistent store.
/// The recommended watchdog frequency is 0.2-0.5 seconds, but a smaller and many larger settings are also available.
/// Writing a zero disables the watchdog.
/// By default watchdog is set to 5, which means a 0.5 second timeout.
/// </summary>
//void SetWatchdogTimeout(float seconds)
//{
//  if (attachCharacteristic(sbrickRemoteControlServiceUUID, sbrickRemoteControlCharacteristicUUID) &&
//    _remoteControlCharacteristic->canWrite()) {
//
//  }
//
//  // Safety check before writing anything (not needed though).
//  if (_characteristic.CharacteristicProperties.HasFlag(GattCharacteristicProperties.Write))
//  {
//    var setWatchDogCommand = new List<byte>{ (byte)Command.SetWatchDogTimeOut, (byte)(seconds * 10) };
//
//    try
//    {
//      var status = await _characteristic.WriteValueAsync(setWatchDogCommand.ToArray().AsBuffer());
//
//      if (status == GattCommunicationStatus.Success)
//      {
//        // ...
//      }
//
//      if (status == GattCommunicationStatus.Unreachable)
//      {
//        // ...
//      }
//    }
//    catch (Exception ex)
//    {
//      throw;
//      //// TODO: Signal error
//    }
//  }
//}

//  void Drive() {
//
//  }
//
//  void EmergencyBreak() {
//    // Reset all channels.
//    Drive(0, 0, 0, 0);
//  }

bool SBrickHubClient::connectToServer()
{
  Serial.print("[" + String(xPortGetCoreID()) + "] SBrick: Connecting to ");
  Serial.println(_address->toString().c_str());

  if (_sbrick == nullptr)
  {
    _sbrick = NimBLEDevice::createClient(*_address);
    //Serial.println(" - Created client");
  }

  if (_clientCallback == nullptr)
  {
    _clientCallback = new SBrickClientCallback(this);
  }
  _sbrick->setClientCallbacks(_clientCallback);

  // Connect to the remote BLE Server.
  if (!_sbrick->connect(false))
  {
    // Serial.println("- Failed to connect to server");
    return false;
  }
  //Serial.println(" - Connected to server");

  // Try to obtain a reference to the remote control characteristic in the remote control service of the BLE server.
  if (!attachCharacteristic(sbrickRemoteControlServiceUUID, sbrickRemoteControlCharacteristicUUID))
  {
    // Failed to find the remote control service or characteristic.
    _sbrick->disconnect();
    return false;
  }

  // Read the value of the characteristic.
  //if(_remoteControlCharacteristic->canRead()) {
  //  std::string value = _remoteControlCharacteristic->readValue();
  //  //Serial.print("The characteristic value was: ");
  //  //Serial.println(value.c_str());
  //}

  //if(_remoteControlCharacteristic->canNotify()) {
  //  _remoteControlCharacteristic->registerForNotify(notifyCallback);
  //}

  return true;
}

// static void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
//   Serial.print("Notify callback for characteristic ");
//   Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
//   Serial.print(" of data length ");
//   Serial.println(length);
//   Serial.print("data: ");
//   Serial.println((char*)pData);
// }

// Private members.
bool SBrickHubClient::attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID)
{
  if (_remoteControlCharacteristic != nullptr)
  {
    return true;
  }

  // Obtain a reference to the service remote control service in the BLE server.
  _remoteControlService = _sbrick->getService(serviceUUID);
  if (_remoteControlService == nullptr)
  {
    return false;
  }

  // Obtain a reference to the remote control characteristic in the remote control service of the BLE server.
  _remoteControlCharacteristic = _remoteControlService->getCharacteristic(characteristicUUID);

  return _remoteControlCharacteristic != nullptr;
}
