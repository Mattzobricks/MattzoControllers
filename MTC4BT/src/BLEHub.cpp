#include <Arduino.h>

#include "BLEHub.h"
#include "BLEDeviceCallbacks.h"
#include "BLEClientCallback.h"
#include "MCLightController.h"
#include "log4MC.h"

BLEHub::BLEHub(BLEHubConfiguration *config, int16_t speedStep, int16_t brakeStep)
{
    _config = config;
    _speedStep = speedStep;
    _brakeStep = brakeStep;

    initChannelControllers();

    _driveTaskHandle = NULL;
    _hub = nullptr;
    _advertisedDeviceCallback = nullptr;
    _clientCallback = nullptr;
    _ebrake = false;
    _blinkLights = false;
    _blinkUntil = 0;
    _isDiscovering = false;
    _isDiscovered = false;
    _isConnected = false;
    _remoteControlService = nullptr;
    _remoteControlCharacteristic = nullptr;
    // _genericAccessCharacteristic = nullptr;
    // _deviceInformationCharacteristic = nullptr;
}

bool BLEHub::IsEnabled()
{
    return _config->Enabled;
}

bool BLEHub::IsDiscovered()
{
    return _isDiscovered;
}

bool BLEHub::IsConnected()
{
    return _isConnected;
}

std::string BLEHub::GetAddress()
{
    return _config->DeviceAddress->toString();
}

void BLEHub::Drive(const int16_t minSpeed, const int16_t speed)
{
    setTargetSpeedPercByAttachedDevice(DeviceType::Motor, minSpeed, speed);
}

void BLEHub::HandleFn(Fn *fn, bool on)
{
    BLEHubChannel channel = bleHubChannelMap()[fn->GetPortConfiguration()->GetAddress()];
    log4MC::vlogf(LOG_DEBUG, "BLE : Hub handling function %u for channel %s.", fn->GetPortConfiguration()->GetAttachedDeviceType(), fn->GetPortConfiguration()->GetAddress().c_str());
    setTargetSpeedPercForChannelByAttachedDevice(channel, fn->GetPortConfiguration()->GetAttachedDeviceType(), 0, on ? _config->LightPerc : 0);
}

void BLEHub::BlinkLights(int durationInMs)
{
    _blinkUntil = millis() + durationInMs;
}

// If true, immediately sets the current speed for all channels to zero.
// If false, releases the emergency brake.
void BLEHub::EmergencyBrake(const bool enabled)
{
    if (enabled == _ebrake)
    {
        // Status hasn't changed. Ignore.
        return;
    }

    // Set hub e-brake status.
    _ebrake = enabled;

    if (_ebrake)
    {
        log4MC::vlogf(LOG_DEBUG, "BLE : Hub %s e-braking on all channels.", _config->DeviceAddress->toString().c_str());

        // Set e-brake on all channels.
        for (int i = 0; i < _channelControllers.size(); i++)
        {
            _channelControllers.at(i)->EmergencyBrake();
        }
    }
    else
    {
        log4MC::vlogf(LOG_DEBUG, "BLE : Hub %s e-brake lifted on all channels.", _config->DeviceAddress->toString().c_str());
    }
}

// Returns a boolean value indicating whether the lights should automatically turn on when the train starts driving.
bool BLEHub::GetAutoLightsEnabled()
{
    return _config->AutoLightsEnabled;
}

bool BLEHub::Connect(const uint8_t watchdogTimeOutInTensOfSeconds)
{
    log4MC::vlogf(LOG_INFO, "BLE : Connecting to hub '%s'...", _config->DeviceAddress->toString().c_str());

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
            log4MC::warn("BLE : Max clients reached - no more connections available.");
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
      * These settings are safe for 3 clients to connect reliably, can go faster if you have less connections. 
      * Timeout should be a multiple of the interval, minimum is 100ms.
      * Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
      */
        _hub->setConnectionParams(12, 12, 0, 51);

        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        _hub->setConnectTimeout(ConnectDelayInSeconds);

        // Connect to the remote BLE Server.
        if (!_hub->connect(_advertisedDevice))
        {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(_hub);
            log4MC::vlogf(LOG_WARNING, "BLE : Failed to connect to hub '%s', deleted client.", _config->DeviceAddress->toString().c_str());
            _isDiscovered = false;
            return false;
        }
    }

    if (!_hub->isConnected())
    {
        if (!_hub->connect(_advertisedDevice))
        {
            log4MC::vlogf(LOG_WARNING, "BLE : Failed to connect to hub '%s'.", _config->DeviceAddress->toString().c_str());
            _isDiscovered = false;
            return false;
        }
    }

    // Try to obtain a reference to the remote control characteristic in the remote control service of the BLE server.
    // If we can set the watchdog timeout, we consider our connection attempt a success.
    if (!SetWatchdogTimeout(watchdogTimeOutInTensOfSeconds))
    {
        // Failed to find the remote control service or characteristic or write/read the value.
        _hub->disconnect();
        return false;
    }

    //if(_remoteControlCharacteristic->canNotify()) {
    //  _remoteControlCharacteristic->registerForNotify(notifyCallback);
    //}

    // Start drive task loop.
    return startDriveTask();
}

void BLEHub::initChannelControllers()
{
    // TODO: This method should be made more robust to prevent config errors, like configuring the same channel twice.

    for (PortConfiguration *config : _config->Channels)
    {
        _channelControllers.push_back(new BLEHubChannelController(config, _speedStep, _brakeStep));
    }

    // log4MC::vlogf(LOG_INFO, "BLE : Hub %s channels initialized.", _config->DeviceAddress->toString().c_str());
}

void BLEHub::setTargetSpeedPercByAttachedDevice(DeviceType device, int16_t minSpeedPerc, int16_t speedPerc)
{
    // Serial.print("Setting ");
    // Serial.print(device);
    // Serial.print(" for ");
    // Serial.print(_channelControllers.size());
    // Serial.println(" channel(s)");

    for (int i = 0; i < _channelControllers.size(); i++)
    {
        BLEHubChannel channel = _channelControllers.at(i)->GetChannel();
        setTargetSpeedPercForChannelByAttachedDevice(channel, device, minSpeedPerc, speedPerc);
    }
}

void BLEHub::setTargetSpeedPercForChannelByAttachedDevice(BLEHubChannel channel, DeviceType device, int16_t minSpeedPerc, int16_t speedPerc)
{
    BLEHubChannelController *controller = findControllerByChannel(channel);
    if (controller != nullptr && controller->GetAttachedDevice() == device)
    {
        controller->SetMinSpeedPerc(minSpeedPerc);
        controller->SetTargetSpeedPerc(speedPerc);
    }
}

uint8_t BLEHub::getRawChannelSpeedForController(BLEHubChannelController *controller)
{
    if ((_blinkUntil > millis() || _ebrake) && controller->GetAttachedDevice() == DeviceType::Light)
    {
        // Force blinking lights when requested or when e-brake is enabled.
        controller->SetCurrentSpeedPerc(MCLightController::Blink() ? _config->LightPerc : 0);
    }

    return MapSpeedPercToRaw(controller->GetCurrentSpeedPerc());
}

BLEHubChannelController *BLEHub::findControllerByChannel(BLEHubChannel channel)
{
    for (BLEHubChannelController *controller : _channelControllers)
    {
        if (controller->GetChannel() == channel)
        {
            return controller;
        }
    }

    return nullptr;
}

bool BLEHub::attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID)
{
    if (_remoteControlCharacteristic != nullptr)
    {
        return true;
    }

    // Obtain a reference to the service remote control service in the BLE server.
    _remoteControlService = _hub->getService(serviceUUID);
    if (_remoteControlService == nullptr)
    {
        return false;
    }

    // Obtain a reference to the remote control characteristic in the remote control service of the BLE server.
    _remoteControlCharacteristic = _remoteControlService->getCharacteristic(characteristicUUID);

    return _remoteControlCharacteristic != nullptr;
}

BaseType_t BLEHub::startDriveTask()
{
    // Determine drive task name.
    // char* taskName = "DT_";
    // strcat(taskName, _deviceName.c_str());

    // Attempt to run drive task, return result.
    return xTaskCreatePinnedToCore(this->driveTaskImpl, "DriveTask", BLE_StackDepth, this, BLE_TaskPriority, &_driveTaskHandle, BLE_CoreID) == pdPASS;
}

void BLEHub::driveTaskImpl(void *_this)
{
    ((BLEHub *)_this)->DriveTaskLoop();
}