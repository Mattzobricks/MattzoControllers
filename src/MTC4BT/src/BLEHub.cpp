#include <Arduino.h>

#include "BLEClientCallback.h"
#include "BLEDeviceCallbacks.h"
#include "BLEHub.h"
#include "MCLightController.h"
#include "log4MC.h"

using namespace std::placeholders;

BLEHub::BLEHub(BLEHubConfiguration *config)
{
    _config = config;

    initChannelControllers();

    _driveTaskHandle = NULL;
    _hub = nullptr;
    _advertisedDeviceCallback = nullptr;
    _clientCallback = nullptr;
    _ebrake = false;
    _blinkLights = false;
    _blinkUntil = 0;
    _isDiscovered = false;
    _isConnected = false;
    _remoteControlService = nullptr;
    _remoteControlCharacteristic = nullptr;
    // _genericAccessCharacteristic = nullptr;
    // _deviceInformationCharacteristic = nullptr;
}

/// @brief return the hub type
/// @return BLEHubType
BLEHubType BLEHub::GetHubType()
{
    return _config->HubType;
}

bool BLEHub::IsDiscovered()
{
    return _isDiscovered;
}

bool BLEHub::IsConnected()
{
    return _isConnected;
}

void BLEHub::SetConnectCallback(std::function<void(bool)> callback)
{
    _onConnectionChangedCallback = callback;
}

std::string BLEHub::GetRawAddress()
{
    return _config->DeviceAddress->toString();
}

NimBLEAddress BLEHub::GetAddress()
{
    return *_config->DeviceAddress;
}

void BLEHub::Drive(const int16_t minPwrPerc, const int16_t pwrPerc)
{
    setTargetPwrPercByAttachedDevice(DeviceType::Motor, minPwrPerc, pwrPerc);
}

int16_t BLEHub::GetCurrentDrivePwrPerc()
{
    for (BLEHubChannelController *controller : _channelControllers) {
        if (controller->GetAttachedDevice() == DeviceType::Motor) {
            return controller->GetCurrentPwrPerc();
        }
    }

    return 0;
}

void BLEHub::Execute(MCLocoAction *action)
{
    BLEHubChannelController *controller = findControllerByChannel(bleHubChannelMap()[action->GetChannel()->GetAddress()]);

    if (controller) {
        if (controller->GetHubChannel() == BLEHubChannel::OnboardLED) {
            controller->SetHubLedColor(action->GetColor());
        } else {
            controller->SetTargetPwrPerc(action->GetTargetPowerPerc());
        }
    }
}

void BLEHub::BlinkLights(int durationInMs)
{
    _blinkUntil = millis() + durationInMs;
}

void BLEHub::SetHubLedColor(HubLedColor color)
{
    BLEHubChannelController *controller = findControllerByChannel(BLEHubChannel::OnboardLED);

    if (controller) {
        controller->SetHubLedColor(color);
    }
}

// If true, immediately sets the current speed for all channels to zero.
// If false, releases the manual brake.
void BLEHub::SetManualBrake(const bool enabled)
{
    if (enabled == _mbrake) {
        // Status hasn't changed. Ignore.
        return;
    }

    // Set hub manual brake status.
    _mbrake = enabled;

    // Set manual brake on all channels.
    for (BLEHubChannelController *channel : _channelControllers) {
        channel->ManualBrake(_mbrake);
    }
}

// If true, immediately sets the current speed for all channels to zero.
// If false, releases the emergency brake.
void BLEHub::SetEmergencyBrake(const bool enabled)
{
    if (enabled == _ebrake) {
        // Status hasn't changed. Ignore.
        return;
    }

    // Set hub e-brake status.
    _ebrake = enabled;

    // Set e-brake on all channels.
    for (BLEHubChannelController *channel : _channelControllers) {
        channel->EmergencyBrake(_ebrake);
    }
}

bool BLEHub::Connect(const uint8_t watchdogTimeOutInTensOfSeconds)
{
    log4MC::vlogf(LOG_INFO, "BLE : Connecting to hub '%s'...", _config->DeviceAddress->toString().c_str());

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getClientListSize()) {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        _hub = NimBLEDevice::getClientByPeerAddress(_advertisedDevice->getAddress());
        if (_hub) {
            if (!_hub->connect(_advertisedDevice, false)) {
                /* Serial.println("Reconnect failed"); */
                _isDiscovered = false;
                return false;
            }

            _isConnected = true;
            log4MC::vlogf(LOG_INFO, "BLE : Reconnected to hub '%s'...", _config->DeviceAddress->toString().c_str());
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
            _hub = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!_hub) {
        if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
            log4MC::warn("BLE : Max clients reached - no more connections available.");
            _isDiscovered = false;
            return false;
        }

        _hub = NimBLEDevice::createClient();

        if (_clientCallback == nullptr) {
            _clientCallback = new BLEClientCallback(this);
        }
        _hub->setClientCallbacks(_clientCallback, false);

        /** Set how long we are willing to wait for the connection to complete (seconds) */
        _hub->setConnectTimeout(ConnectDelayInSeconds);

        // Connect to the remote BLE Server.
        if (!_hub->connect(_advertisedDevice)) {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(_hub);
            log4MC::vlogf(LOG_WARNING, "BLE : Failed to connect to hub '%s', deleted client.", _config->DeviceAddress->toString().c_str());
            _isDiscovered = false;
            return false;
        }
    }

    if (!_hub->isConnected()) {
        if (!_hub->connect(_advertisedDevice)) {
            log4MC::vlogf(LOG_WARNING, "BLE : Failed to connect to hub '%s'.", _config->DeviceAddress->toString().c_str());
            _isDiscovered = false;
            return false;
        }
    }

    // Try to obtain a reference to the remote control characteristic in the remote control service of the BLE server.
    // If we can set the watchdog timeout, we consider our connection attempt a success.
    if (!SetWatchdogTimeout(watchdogTimeOutInTensOfSeconds)) {
        // Failed to find the remote control service or characteristic or write/read the value.
        _hub->disconnect();
        return false;
    }

    // Subscribe to receive callback notifications.
    if (_remoteControlCharacteristic->canNotify()) {
        _remoteControlCharacteristic->subscribe(true, std::bind(&BLEHub::NotifyCallback, this, _1, _2, _3, _4), true);
    }

    // Start drive task loop.
    return startDriveTask();
}

void BLEHub::initChannelControllers()
{
    // TODO: This method should be made more robust to prevent config errors, like configuring the same channel twice.

    for (MCChannelConfig *config : _config->Channels) {
        _channelControllers.push_back(new BLEHubChannelController(config));
    }

    // log4MC::vlogf(LOG_INFO, "BLE : Hub %s channels initialized.", _config->DeviceAddress->toString().c_str());
}

void BLEHub::setTargetPwrPercByAttachedDevice(DeviceType device, int16_t minPwrPerc, int16_t pwrPerc)
{
    for (BLEHubChannelController *channel : _channelControllers) {
        if (channel->GetAttachedDevice() == device) {
            channel->SetTargetPwrPerc(pwrPerc);
            channel->SetMinPwrPerc(minPwrPerc);
        }
    }
}

HubLedColor BLEHub::getRawLedColorForController(BLEHubChannelController *controller)
{
    if ((_ebrake || _blinkUntil > millis()) && controller->GetAttachedDevice() == DeviceType::Light) {
        // Force blinking LED (white when on, black when off) when requested.
        return MCLightController::Blink() ? controller->GetHubLedColor() : HubLedColor::BLACK;
    }

    return controller->GetHubLedColor();
}

uint8_t BLEHub::getRawChannelPwrForController(BLEHubChannelController *controller)
{
    if ((_ebrake || _blinkUntil > millis()) && controller->GetAttachedDevice() == DeviceType::Light) {
        // Force blinking lights (50% when on, 0% when off) when requested.
        return MCLightController::Blink() ? 50 : 0;
    }

    return MapPwrPercToRaw(controller->GetCurrentPwrPerc());
}

BLEHubChannelController *BLEHub::findControllerByChannel(BLEHubChannel channel)
{
    for (BLEHubChannelController *controller : _channelControllers) {
        if (controller->GetHubChannel() == channel) {
            return controller;
        }
    }

    return nullptr;
}

bool BLEHub::attachCharacteristic(NimBLEUUID serviceUUID, NimBLEUUID characteristicUUID)
{
    if (_remoteControlCharacteristic != nullptr) {
        return true;
    }

    // Obtain a reference to the service remote control service in the BLE server.
    _remoteControlService = _hub->getService(serviceUUID);
    if (_remoteControlService == nullptr) {
        return false;
    }

    // Obtain a reference to the remote control characteristic in the remote control service of the BLE server.
    _remoteControlCharacteristic = _remoteControlService->getCharacteristic(characteristicUUID);
    return _remoteControlCharacteristic != nullptr;
}

bool BLEHub::startDriveTask()
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

void BLEHub::connected()
{
    this->_isConnected = true;
    if (this->_onConnectionChangedCallback) {
        this->_onConnectionChangedCallback(true);
    }
}

void BLEHub::disconnected()
{
    this->_isConnected = false;
    if (this->_onConnectionChangedCallback) {
        this->_onConnectionChangedCallback(false);
    }
}

// dump in hex the pdata block, this should ease programming
// new BLE devices, or add possibilities/capabilities of existing
// hubs
void BLEHub::dumpPData(uint8_t *pData, size_t length)
{
    char buffer[17 * 3 + 25];
    char asciiBuffer[18];
    char byteBuffer[5];
    size_t index; // needed outside the loop!
    for (index = 0; index < length; index++) {
        if ((index % 16) == 0) {
            if (index != 0) {
                log4MC::vlogf(LOG_DEBUG, "%s %s", buffer, asciiBuffer);
            }
            // Output the offset.
            snprintf(buffer, 6, "%04x ", index);
        }
        // Output the byte value.
        snprintf(byteBuffer, 4, "%02x ", pData[index]);
        strncat(buffer, byteBuffer, 4);
        if ((pData[index] < 0x20) || (pData[index] > 0x7e)) {
            asciiBuffer[index % 16] = '.';
        } else {
            asciiBuffer[index % 16] = pData[index];
        }
        asciiBuffer[(index % 16) + 1] = '\0';
    }
    // Pad out last line if not exactly 16 characters.
    while ((index % 16) != 0) {
        strncat(buffer, "   ", 4);
        index++;
    }

    // And print the final ASCII bit.
    log4MC::vlogf(LOG_DEBUG, "%s '%s'", buffer, asciiBuffer);
}