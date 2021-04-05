#include <Arduino.h>

#include "BLEClientCallback.h"
#include "PUHub.h"

#define MAX_PUHUB_CHANNEL_COUNT 2

static BLEUUID remoteControlServiceUUID(PU_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(PU_REMOTECONTROL_CHARACTERISTIC_UUID);

PUHub::PUHub(std::string deviceName, std::string deviceAddress, std::vector<ChannelConfiguration> channels[], int16_t lightPerc, bool autoLightsOnEnabled, bool enabled)
    : BLEHub(deviceName, deviceAddress, channels, lightPerc, autoLightsOnEnabled, enabled)
{
}

bool PUHub::Connect(const uint8_t watchdogTimeOutInTensOfSeconds)
{
  Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Connecting to ");
  Serial.println(_address->toString().c_str());

  /** Check if we have a client we should reuse first **/
  if (NimBLEDevice::getClientListSize())
  {
    /** Special case when we already know this device, we send false as the
     *  second argument in connect() to prevent refreshing the service database.
     *  This saves considerable time and power.
     */
    _hub = NimBLEDevice::getClientByPeerAddress(_advertisedDevice->getAddress());
    if (_hub)
    {
      if (!_hub->connect(_advertisedDevice, false))
      {
        /* Serial.println("Reconnect failed"); */
        _isDiscovered = false;
        return false;
      }
      /* Serial.println("Reconnected client"); */
    }
    /** We don't already have a client that knows this device,
     *  we will check for a client that is disconnected that we can use.
     */
    else
    {
      _hub = NimBLEDevice::getDisconnectedClient();
    }
  }

  /** No client to reuse? Create a new one. */
  if (!_hub)
  {
    if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
    {
      Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Max clients reached - no more connections available");
      _isDiscovered = false;
      return false;
    }

    _hub = NimBLEDevice::createClient();

    if (_clientCallback == nullptr)
    {
      _clientCallback = new BLEClientCallback(this);
    }
    _hub->setClientCallbacks(_clientCallback, false);

    /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
     *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
     *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
     *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
     */
    _hub->setConnectionParams(12, 12, 0, 51);

    /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
    _hub->setConnectTimeout(ConnectDelayInSeconds);

    // Connect to the remote BLE Server.
    if (!_hub->connect(_advertisedDevice))
    {
      /** Created a client but failed to connect, don't need to keep it as it has no data */
      NimBLEDevice::deleteClient(_hub);
      Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Failed to connect, deleted client");
      _isDiscovered = false;
      return false;
    }
  }

  if (!_hub->isConnected())
  {
    if (!_hub->connect(_advertisedDevice))
    {
      Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Failed to connect");
      _isDiscovered = false;
      return false;
    }
  }

  //Serial.println(" - Connected to server");

  // Try to obtain a reference to the remote control characteristic in the remote control service of the BLE server.
  // If we can set the watchdog timeout, we consider our connection attempt a success.
  if (!setWatchdogTimeout(watchdogTimeOutInTensOfSeconds))
  {
    // Failed to find the remote control service or characteristic or write/read the value.
    _hub->disconnect();
    return false;
  }

  //if(_remoteControlCharacteristic->canNotify()) {
  //  _remoteControlCharacteristic->registerForNotify(notifyCallback);
  //}

  // Start drive task loop.
  return StartDriveTask();
}

void PUHub::DriveTaskLoop()
{
  for (;;)
  {
    for (int channel = 0; channel < _channelControllers.size(); channel++)
    {
      // Update current channel speeds, if we're not emergency breaking.
      if (_ebreak || _channelControllers.at(channel)->UpdateCurrentSpeedPerc())
      {
        // Serial.print(channel);
        // Serial.print(": rawspd=");
        // Serial.println(MapSpeedPercToRaw(_channelControllers.at(channel)->GetCurrentSpeedPerc()));

        // Construct drive command.
        byte targetSpeed = MapSpeedPercToRaw(_channelControllers.at(channel)->GetCurrentSpeedPerc());
        byte setMotorCommand[8] = {0x81, (byte)channel, 0x11, 0x51, 0x00, targetSpeed};
        int size = 6;

        byte byteCmd[size + 2] = {(byte)(size + 2), 0x00};
        memcpy(byteCmd + 2, setMotorCommand, size);

        // Send drive command.
        if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), false))
        {
          Serial.println("Drive failed");
        }
      }
    }

    // Wait half the watchdog timeout (converted from s/10 to s/1000).
    vTaskDelay(_watchdogTimeOutInTensOfSeconds * 50 / portTICK_PERIOD_MS);
  }
}

int16_t PUHub::MapSpeedPercToRaw(int speedPerc)
{
  if (speedPerc == 0)
  {
    return 0; // 0 = float, 127 = stop motor
  }

  if (speedPerc > 0)
  {
    return map(speedPerc, 0, 100, PU_MIN_SPEED_FORWARD, PU_MAX_SPEED_FORWARD);
  }

  return map(abs(speedPerc), 0, 100, PU_MIN_SPEED_REVERSE, PU_MAX_SPEED_REVERSE);
}

bool PUHub::setWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
{
  _watchdogTimeOutInTensOfSeconds = watchdogTimeOutInTensOfSeconds;

  if (!attachCharacteristic(remoteControlServiceUUID, remoteControlCharacteristicUUID))
  {
    return false;
  }

  if (!_remoteControlCharacteristic->canWrite())
  {
    return false;
  }

  Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Watchdog timeout successfully set to s/10: ");
  Serial.println(_watchdogTimeOutInTensOfSeconds);

  return true;
}