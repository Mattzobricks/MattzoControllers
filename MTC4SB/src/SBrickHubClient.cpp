#include <Arduino.h>

#include "SBrickHubClient.h"
#include "NimBLEDevice.h"
#include "SBrickConst.h"
#include "SBrickClientCallback.h"
#include "SBrickAdvertisedDeviceCallbacks.h"

/*
*   - subscribeToNotifications(callback)
*       pMyRemoteCharacteristic->registerForNotify(notifyCallback)
*       void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t length, bool isNotify)
* SBrickHub.DriveChannel(channel, command)
* SBrickHub.EmergencyBreak()
*/

static BLEUUID sbrickRemoteControlServiceUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
static BLEUUID sbrickRemoteControlCharacteristicUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);

const int8_t BRAKE = 0;
const int8_t DRIVE = 1;
const int8_t SET_WATCHDOG_TIMEOUT = 13;
const int8_t GET_WATCHDOG_TIMEOUT = 14;
const int8_t BREAK_WITH_PM = 19;
const int8_t GET_CHANNEL_STATUS = 34;

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

  Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Scan started");
  // Set the callback we want to use to be informed when we have detected a new device.
  if (_advertisedDeviceCallback == nullptr)
  {
    _advertisedDeviceCallback = new SBrickAdvertisedDeviceCallbacks(this);
  }
  scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
  scanner->start(scanDurationInSeconds, false);
  Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Scan stopped");

  _isDiscovering = false;
}

/// <summary>
/// Connect to the SBrick Hub.
/// </summary>
bool SBrickHubClient::Connect(const uint32_t watchdogTimeOutInMs)
{
  return connectToServer(watchdogTimeOutInMs);
}

bool SBrickHubClient::IsDiscovered()
{
  return _isDiscovered;
}

bool SBrickHubClient::IsConnected()
{
  return _isConnected;
}

bool SBrickHubClient::Drive(const int8_t powerA, const int8_t powerB, const int8_t powerC, const int8_t powerD) {
  if (!attachCharacteristic(sbrickRemoteControlServiceUUID, sbrickRemoteControlCharacteristicUUID)){
    return false;
  }

  if(!_remoteControlCharacteristic->canWrite()) {
    return false;
  }

  uint8_t byteWrite[13] = { DRIVE, 0, 1, powerA, 1, 1, powerB, 2, 1, powerC, 3, 1, powerD };
  if(!_remoteControlCharacteristic->writeValue(byteWrite, sizeof(byteWrite), false)) {
    return false;
  }
  Serial.println("success");
  
  return true;
}

std::string SBrickHubClient::getDeviceName()
{
  return _deviceName;
}

bool SBrickHubClient::connectToServer(const uint8_t watchdogTimeOutInMs)
{
  Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Connecting to ");
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

  /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
   *  These settings are safe for 3 clients to connect reliably, can go faster if you have less connections. 
   *  Timeout should be a multiple of the interval, minimum is 100ms.
   *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
   */
  //_sbrick->setConnectionParams(12, 12, 0, 51);

  /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
  _sbrick->setConnectTimeout(5);

  // Connect to the remote BLE Server.
  if (!_sbrick->connect(false))
  {
    // Serial.println("- Failed to connect to server");
    _isDiscovered = false;
    return false;
  }
  //Serial.println(" - Connected to server");

  // Try to obtain a reference to the remote control characteristic in the remote control service of the BLE server.
  // If we can set the watchdog timeout, we consider our connection attempt a success.
  if (!setWatchdogTimeout(watchdogTimeOutInMs))
  {
    // Failed to find the remote control service or characteristic or write/read the value.
    _sbrick->disconnect();
    return false;
  }

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
bool SBrickHubClient::setWatchdogTimeout(const uint8_t watchdogTimeOutInMs) {
  if (!attachCharacteristic(sbrickRemoteControlServiceUUID, sbrickRemoteControlCharacteristicUUID)){
    return false;
  }

  if(!_remoteControlCharacteristic->canWrite()) {
    return false;
  }

  uint8_t byteWrite[2] = { SET_WATCHDOG_TIMEOUT, watchdogTimeOutInMs };
  if(!_remoteControlCharacteristic->writeValue(byteWrite, sizeof(byteWrite), false)) {
    return false;
  }

  uint8_t byteRead[1] = { GET_WATCHDOG_TIMEOUT };
  if(!_remoteControlCharacteristic->writeValue(byteRead, sizeof(byteRead), false)) {
    return false;
  }

  Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Watchdog timeout successfully set to s/10: ");
  Serial.println(_remoteControlCharacteristic->readValue<uint8_t>());

  return true;
}

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
