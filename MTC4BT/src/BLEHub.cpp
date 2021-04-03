#include <Arduino.h>

#include "NimBLEDevice.h"

#include "BLEHub.h"
#include "AdvertisedBLEDeviceCallbacks.h"

BLEHub::BLEHub(std::string deviceName, std::string deviceAddress, std::vector<ChannelConfiguration> channels[], int16_t lightPerc, bool autoLightsEnabled, bool enabled)
{
    _deviceName = deviceName;
    _address = new NimBLEAddress(deviceAddress);
    _lightPerc = lightPerc;
    _autoLightsEnabled = autoLightsEnabled;
    _isEnabled = enabled;

    initChannelControllers(channels);

    _driveTaskHandle = NULL;
    _hub = nullptr;
    _advertisedDeviceCallback = nullptr;
    _clientCallback = nullptr;
    _ebreak = false;
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
    return _isEnabled;
}

void BLEHub::StartDiscovery(NimBLEScan *scanner, const uint32_t scanDurationInSeconds)
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
        _advertisedDeviceCallback = new AdvertisedBLEDeviceCallbacks(this);
    }
    scanner->setAdvertisedDeviceCallbacks(_advertisedDeviceCallback);
    scanner->start(scanDurationInSeconds, false);
    Serial.println("[" + String(xPortGetCoreID()) + "] BLE : Scanning for " + _deviceName.c_str() + " aborted.");

    _isDiscovering = false;
}

bool BLEHub::IsDiscovered()
{
    return _isDiscovered;
}

bool BLEHub::IsConnected()
{
    return _isConnected;
}

BaseType_t BLEHub::StartDriveTask()
{
    // Determine drive task name.
    // char* taskName = "DT_";
    // strcat(taskName, _deviceName.c_str());

    // Attempt to run drive task, return result.
    return xTaskCreatePinnedToCore(this->driveTaskImpl, "DriveTask", BLE_StackDepth, this, BLE_TaskPriority, &_driveTaskHandle, BLE_CoreID) == pdPASS;
}

void BLEHub::Drive(const int16_t speedPerc)
{
    setTargetSpeedPercByAttachedDevice(AttachedDevice::MOTOR, speedPerc);
}

void BLEHub::SetLights(bool on)
{
    setTargetSpeedPercByAttachedDevice(AttachedDevice::LIGHT, on ? _lightPerc : 0);
}

void BLEHub::SetLights(HubChannel channel, bool on)
{
    // Serial.print("Turn lights ");
    // Serial.print(on ? "on" : "off");
    // Serial.print(" for channel ");
    // Serial.println(channel);
    setTargetSpeedPercForChannelByAttachedDevice(channel, AttachedDevice::LIGHT, on ? _lightPerc : 0);
}

// If true, immediately sets the current speed for all channels to zero.
// If false, releases the emergency break.
void BLEHub::EmergencyBreak(const bool enabled)
{
    // Set e-break status.
    _ebreak = enabled;

    if (_ebreak)
    {
        Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Emergency breaking on channels: ");
        Serial.println(_channelControllers.size());
    }
    else
    {
        Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Emergency breaking lifted on channels: ");
        Serial.println(_channelControllers.size());
    }

    if (_ebreak)
    {
        for (int i = 0; i < _channelControllers.size(); i++)
        {
            ChannelController *controller = _channelControllers.at(i);
            controller->EmergencyBreak();
        }
    }
}

// Returns the device name.
std::string BLEHub::GetDeviceName()
{
    return _deviceName;
}

// Returns a boolean value indicating whether the lights should automatically turn on when the train starts driving.
bool BLEHub::GetAutoLightsEnabled()
{
    return _autoLightsEnabled;
}

void BLEHub::initChannelControllers(std::vector<ChannelConfiguration> channels[])
{
    // TODO: This method should be made more robust to prevent config errors, like configuring the same channel twice.

    for (int i = 0; i < channels->size(); i++)
    {
        ChannelConfiguration config = channels->at(i);

        // Serial.print("Custructing controller for channel: ");
        // Serial.print(config.channel);
        // Serial.print(", device: ");
        // Serial.print(config.device);
        // Serial.print(", min: ");
        // Serial.print(-254);
        // Serial.print(", max: ");
        // Serial.print(254);
        // Serial.print(", speedStep: ");
        // Serial.print(config.speedStep);
        // Serial.print(", brakeStep: ");
        // Serial.print(config.brakeStep);
        // Serial.println();

        _channelControllers.push_back(new ChannelController(config.channel, config.device, config.speedStep, config.brakeStep));
    }
}

void BLEHub::setTargetSpeedPercByAttachedDevice(AttachedDevice device, int16_t speedPerc)
{
    for (int i = 0; i < _channelControllers.size(); i++)
    {
        HubChannel channel = static_cast<HubChannel>(i);
        setTargetSpeedPercForChannelByAttachedDevice(channel, device, speedPerc);
    }
}

void BLEHub::setTargetSpeedPercForChannelByAttachedDevice(HubChannel channel, AttachedDevice device, int16_t speedPerc)
{
    ChannelController *controller = findControllerByChannel(channel);
    if (controller != nullptr && controller->GetAttachedDevice() == device)
    {
        controller->SetTargetSpeedPerc(speedPerc);
    }
}

ChannelController *BLEHub::findControllerByChannel(HubChannel channel)
{
    for (int i = 0; i < _channelControllers.size(); i++)
    {
        if(_channelControllers.at(i)->GetChannel() == channel)
        {
            return _channelControllers.at(i);
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

void BLEHub::driveTaskImpl(void *_this)
{
    ((BLEHub *)_this)->DriveTaskLoop();
}