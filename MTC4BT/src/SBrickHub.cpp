#include <Arduino.h>

#include "SBrickHub.h"
#include "log4MC.h"

#define MAX_SBRICK_CHANNEL_COUNT 4

static BLEUUID remoteControlServiceUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);

const int8_t CMD_BRAKE = 0;
const int8_t CMD_DRIVE = 1;
const int8_t CMD_SET_WATCHDOG_TIMEOUT = 13;
const int8_t CMD_GET_WATCHDOG_TIMEOUT = 14;
const int8_t CMD_BRAKE_WITH_PM = 19;
const int8_t CMD_GET_CHANNEL_STATUS = 34;
const int16_t SBRICK_MAX_CHANNEL_SPEED = 254;
const int16_t SBRICK_MIN_CHANNEL_SPEED = -254;

SBrickHub::SBrickHub(BLEHubConfiguration *config, int16_t speedStep, int16_t brakeStep)
    : BLEHub(config, speedStep, brakeStep)
{
}

bool SBrickHub::SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
{
    _watchdogTimeOutInTensOfSeconds = watchdogTimeOutInTensOfSeconds;

    if (!attachCharacteristic(remoteControlServiceUUID, remoteControlCharacteristicUUID))
    {
        log4MC::error("BLE : Unable to attach to remote control service.");
        return false;
    }

    if (!_remoteControlCharacteristic->canWrite())
    {
        log4MC::error("BLE : Remote control characteristic doesn't allow writing.");
        return false;
    }

    uint8_t byteWrite[2] = {CMD_SET_WATCHDOG_TIMEOUT, watchdogTimeOutInTensOfSeconds};
    if (!_remoteControlCharacteristic->writeValue(byteWrite, sizeof(byteWrite), false))
    {
        log4MC::error("BLE : Writing remote control characteristic failed.");
        return false;
    }

    uint8_t byteRead[1] = {CMD_GET_WATCHDOG_TIMEOUT};
    if (!_remoteControlCharacteristic->writeValue(byteRead, sizeof(byteRead), false))
    {
        log4MC::error("BLE : Writing remote control characteristic failed.");
        return false;
    }

    log4MC::vlogf(LOG_INFO, "BLE : Watchdog timeout successfully set to s/10: ", _remoteControlCharacteristic->readValue<uint8_t>());

    return true;
}

void SBrickHub::DriveTaskLoop()
{
    for (;;)
    {
        if (!_ebrake)
        {
            // Update current channel speeds, if needed.
            for (int channel = 0; channel < _channelControllers.size(); channel++)
            {
                _channelControllers.at(channel)->UpdateCurrentSpeedPerc();
            }
        }

        // Construct drive command.
        uint8_t byteCmd[13] = {
            CMD_DRIVE,
            BLEHubChannel::A,
            channelIsDrivingForward(BLEHubChannel::A),
            getRawChannelSpeed(BLEHubChannel::A),
            BLEHubChannel::B,
            channelIsDrivingForward(BLEHubChannel::B),
            getRawChannelSpeed(BLEHubChannel::B),
            BLEHubChannel::C,
            channelIsDrivingForward(BLEHubChannel::C),
            getRawChannelSpeed(BLEHubChannel::C),
            BLEHubChannel::D,
            channelIsDrivingForward(BLEHubChannel::D),
            getRawChannelSpeed(BLEHubChannel::D)};

        // Send drive command.
        if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), false))
        {
            log4MC::error("BLE : Drive failed. Unabled to write to characteristic.");
        }

        // Wait half the watchdog timeout (converted from s/10 to s/1000).
        //vTaskDelay(_watchdogTimeOutInTensOfSeconds * 50 / portTICK_PERIOD_MS);

        // Wait 50 milliseconds.
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

int16_t SBrickHub::MapSpeedPercToRaw(int speedPerc)
{
    // Map absolute speed (no matter the direction) to raw channel speed.
    return map(abs(speedPerc), 0, 100, 0, SBRICK_MAX_CHANNEL_SPEED);
}

void SBrickHub::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    // TODO: Ignored for now...
}

std::array<uint8_t, 3> SBrickHub::getDriveCommand(BLEHubChannel channel)
{
    BLEHubChannelController *controller = findControllerByChannel(channel);

    std::array<uint8_t, 3> cmd;
    cmd[0] = channel;
    cmd[1] = controller ? controller->IsDrivingForward() : false;
    cmd[2] = controller ? MapSpeedPercToRaw(controller->GetCurrentSpeedPerc()) : 0;

    return cmd;
}

bool SBrickHub::channelIsDrivingForward(BLEHubChannel channel)
{
    BLEHubChannelController *controller = findControllerByChannel(channel);
    return controller ? controller->IsDrivingForward() : false;
}

uint8_t SBrickHub::getRawChannelSpeed(BLEHubChannel channel)
{
    BLEHubChannelController *controller = findControllerByChannel(channel);
    return controller ? getRawChannelSpeedForController(controller) : 0;
}