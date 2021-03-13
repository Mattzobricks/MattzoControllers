#include <Arduino.h>

#include "SBrickHubClient.h"
#include "SBrickHubChannel.h"
#include "NimBLEDevice.h"
#include "SBrickConst.h"
#include "SBrickClientCallback.h"
#include "SBrickAdvertisedDeviceCallbacks.h"

/*
*   - subscribeToNotifications(callback)
*       pMyRemoteCharacteristic->registerForNotify(notifyCallback)
*       void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t length, bool isNotify)
* SBrickHub.EmergencyBreak()
*/

static BLEUUID sbrickRemoteControlServiceUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
static BLEUUID sbrickRemoteControlCharacteristicUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);

const int8_t CMD_BRAKE = 0;
const int8_t CMD_DRIVE = 1;
const int8_t CMD_SET_WATCHDOG_TIMEOUT = 13;
const int8_t CMD_GET_WATCHDOG_TIMEOUT = 14;
const int8_t CMD_BREAK_WITH_PM = 19;
const int8_t CMD_GET_CHANNEL_STATUS = 34;

// Public members

SBrickHubClient::SBrickHubClient(std::string deviceName, std::string deviceAddress, bool enabled)
{
  _driveTaskHandle = NULL;
  _deviceName = deviceName;
  _address = new NimBLEAddress(deviceAddress);
  _sbrick = nullptr;
  _advertisedDeviceCallback = nullptr;
  _clientCallback = nullptr;
  _isEnabled = enabled;
  _ebreak = false;
  _isDiscovering = false;
  _isDiscovered = false;
  _isConnected = false;
  _remoteControlService = nullptr;
  _remoteControlCharacteristic = nullptr;
  _genericAccessCharacteristic = nullptr;
  _deviceInformationCharacteristic = nullptr;
}

/// <summary>
/// Returns a boolean value indicating whether we should attempt to discover and connect to this SBrick Hub.
/// </summary>
bool SBrickHubClient::IsEnabled()
{
  return _isEnabled;
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

  Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Scanning for " + _deviceName.c_str() + "...");
  // Set the callback we want to use to be informed when we have detected a new device.
  if (_advertisedDeviceCallback == nullptr)
  {
    _advertisedDeviceCallback = new SBrickAdvertisedDeviceCallbacks(this);
  }
  scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
  scanner->start(scanDurationInSeconds, false);
  Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Scanning for " + _deviceName.c_str() + " aborted.");

  _isDiscovering = false;
}

/// <summary>
/// Connect to the SBrick Hub.
/// </summary>
bool SBrickHubClient::Connect(const uint32_t watchdogTimeOutInTensOfSeconds)
{
  return connectToServer(watchdogTimeOutInTensOfSeconds);
}

bool SBrickHubClient::IsDiscovered()
{
  return _isDiscovered;
}

bool SBrickHubClient::IsConnected()
{
  return _isConnected;
}

void SBrickHubClient::Drive(const int16_t a, const int16_t b, const int16_t c, const int16_t d)
{
  // Set each channel target speeds.
  DriveChannel(SBrickHubChannel::A, a);
  DriveChannel(SBrickHubChannel::B, b);
  DriveChannel(SBrickHubChannel::C, c);
  DriveChannel(SBrickHubChannel::D, d);
}

void SBrickHubClient::DriveChannel(const SBrickHubChannel::SBrickChannel channel, const int16_t speed)
{
  _channels[channel]->SetTargetSpeed(speed);
}

void SBrickHubClient::EmergencyBreak(const bool enabled)
{
  _ebreak = enabled;

  if (_ebreak)
  {
    // Immediately set each channel speed to zero.
    _channels[SBrickHubChannel::A]->SetSpeed(0);
    _channels[SBrickHubChannel::B]->SetSpeed(0);
    _channels[SBrickHubChannel::C]->SetSpeed(0);
    _channels[SBrickHubChannel::D]->SetSpeed(0);
  }
}

std::string SBrickHubClient::getDeviceName()
{
  return _deviceName;
}

bool SBrickHubClient::connectToServer(const uint16_t watchdogTimeOutInTensOfSeconds)
{
  Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Connecting to ");
  Serial.println(_address->toString().c_str());

  if (_sbrick == nullptr)
  {
    _sbrick = NimBLEDevice::createClient(*_address);
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
  _sbrick->setConnectTimeout(ConnectDelayInSeconds);

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
  if (!setWatchdogTimeout(watchdogTimeOutInTensOfSeconds))
  {
    // Failed to find the remote control service or characteristic or write/read the value.
    _sbrick->disconnect();
    return false;
  }

  //if(_remoteControlCharacteristic->canNotify()) {
  //  _remoteControlCharacteristic->registerForNotify(notifyCallback);
  //}

  // Start drive task loop.
  return startDriveTask();
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
bool SBrickHubClient::setWatchdogTimeout(const uint16_t watchdogTimeOutInTensOfSeconds)
{
  _watchdogTimeOutInTensOfSeconds = watchdogTimeOutInTensOfSeconds;

  if (!attachCharacteristic(sbrickRemoteControlServiceUUID, sbrickRemoteControlCharacteristicUUID))
  {
    return false;
  }

  if (!_remoteControlCharacteristic->canWrite())
  {
    return false;
  }

  uint8_t byteWrite[2] = {CMD_SET_WATCHDOG_TIMEOUT, watchdogTimeOutInTensOfSeconds};
  if (!_remoteControlCharacteristic->writeValue(byteWrite, sizeof(byteWrite), false))
  {
    return false;
  }

  uint8_t byteRead[1] = {CMD_GET_WATCHDOG_TIMEOUT};
  if (!_remoteControlCharacteristic->writeValue(byteRead, sizeof(byteRead), false))
  {
    return false;
  }

  Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Watchdog timeout successfully set to s/10: ");
  Serial.println(_remoteControlCharacteristic->readValue<uint8_t>());

  return true;
}

BaseType_t SBrickHubClient::startDriveTask()
{
  // Determine drive task name.
  // char* taskName = "DT_";
  // strcat(taskName, _deviceName.c_str());

  // Attempt to run drive task, return result.
  return xTaskCreatePinnedToCore(this->driveTaskImpl, "DriveTask", StackDepth, this, TaskPriority, &_driveTaskHandle, CoreID) == pdPASS;
}

void SBrickHubClient::driveTaskImpl(void *_this)
{
  ((SBrickHubClient *)_this)->driveTaskLoop();
}

void SBrickHubClient::driveTaskLoop()
{
  for (;;)
  {
    if (!_ebreak)
    {
      for (int channel = SBrickHubChannel::A; channel != SBrickHubChannel::D + 1; channel++)
      {
        // Serial.print("Channel ");
        // Serial.print(channel);
        // Serial.print(": dir=");
        // Serial.print(_channels[channel]->GetCurrentTargetDirection());
        // Serial.print(" cur=");
        // Serial.print(_channels[channel]->GetCurrentTargetSpeed());

        int8_t dirMultiplier = _channels[channel]->GetCurrentTargetDirection() ? 1 : -1;
        int16_t newTargetSpeed = (_channels[channel]->GetCurrentTargetSpeed() + 10) * dirMultiplier;

        // Serial.print(" stp=");
        // Serial.print(speedStep);

        if (!_channels[channel]->IsAtSetTargetSpeed())
        {
          // Adjust channel target speed with one speed step towards the set target speed.
          // Serial.print(" tar=");
          // Serial.print(newTargetSpeed);
          _channels[channel]->SetCurrentTargetSpeed(newTargetSpeed);
        }

        // Serial.println();
      }

      // Construct drive command.
      uint8_t byteCmd[13] = {
          CMD_DRIVE,
          SBrickHubChannel::A,
          _channels[SBrickHubChannel::A]->GetCurrentTargetDirection(),
          _channels[SBrickHubChannel::A]->GetCurrentTargetSpeed(),
          SBrickHubChannel::B,
          _channels[SBrickHubChannel::B]->GetCurrentTargetDirection(),
          _channels[SBrickHubChannel::B]->GetCurrentTargetSpeed(),
          SBrickHubChannel::C,
          _channels[SBrickHubChannel::C]->GetCurrentTargetDirection(),
          _channels[SBrickHubChannel::C]->GetCurrentTargetSpeed(),
          SBrickHubChannel::D,
          _channels[SBrickHubChannel::D]->GetCurrentTargetDirection(),
          _channels[SBrickHubChannel::D]->GetCurrentTargetSpeed()};

      // Send drive command.
      if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), false))
      {
        Serial.println("Drive failed");
      }
    }

    // Wait half the watchdog timeout (converted from s/10 to s/1000).
    vTaskDelay(_watchdogTimeOutInTensOfSeconds * 50 / portTICK_PERIOD_MS);
  }
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

// Initialize static members.
uint8_t SBrickHubClient::TaskPriority = 1;
int8_t SBrickHubClient::CoreID = 1;
uint32_t SBrickHubClient::StackDepth = 2048;
uint8_t SBrickHubClient::ConnectDelayInSeconds = 30;