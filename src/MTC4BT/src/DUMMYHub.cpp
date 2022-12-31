#include <Arduino.h>

#include "DUMMYHub.h"
#include "log4MC.h"

#define DUMMY_REMOTECONTROL_SERVICE_UUID "00001623-1212-efde-1623-785feabcd123"
#define DUMMY_REMOTECONTROL_CHARACTERISTIC_UUID "00001624-1212-efde-1623-785feabcd123"

static BLEUUID remoteControlServiceUUID(DUMMY_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(DUMMY_REMOTECONTROL_CHARACTERISTIC_UUID);

// COPY of SBrickHup,

const int8_t CMD_BRAKE = 0;
const int8_t CMD_DRIVE = 1;
const int8_t CMD_SET_WATCHDOG_TIMEOUT = 13;
const int8_t CMD_GET_WATCHDOG_TIMEOUT = 14;
const int8_t CMD_BRAKE_WITH_PM = 19;
const int8_t CMD_GET_CHANNEL_STATUS = 34;
const int16_t DUMMY_MAX_CHANNEL_SPEED = 254;

DUMMYHub::DUMMYHub(BLEHubConfiguration *config)
    : BLEHub(config)
{
}

bool DUMMYHub::SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
{
    _watchdogTimeOutInTensOfSeconds = watchdogTimeOutInTensOfSeconds;

    if (!attachCharacteristic(remoteControlServiceUUID, remoteControlCharacteristicUUID)) {
        log4MC::error("DUMMY : Unable to attach to remote control service.");
        return false;
    }

    if (!_remoteControlCharacteristic->canWrite()) {
        log4MC::error("DUMMY : Remote control characteristic doesn't allow writing.");
        return false;
    }

    log4MC::vlogf(LOG_INFO, "DUMMY : Watchdog timeout not set for PU hubs");

    return true;
}

void DUMMYHub::DriveTaskLoop()
{
    bool channelAForward = false;
    bool channelBForward = false;
    bool channelCForward = false;
    bool channelDForward = false;

    uint8_t channelAPwr = 0;
    uint8_t channelBPwr = 0;
    uint8_t channelCPwr = 0;
    uint8_t channelDPwr = 0;

    bool motorFound = false;
    int16_t currentSpeedPerc = 0;
    int16_t targetSpeedPerc = 0;
    log4MC::vlogf(LOG_DEBUG, "DUM : Entering DriveTaskLoop"); 

    for (;;) {
        motorFound = false;
        currentSpeedPerc = 0;
        targetSpeedPerc = 0;
        for (BLEHubChannelController *channel : _channelControllers) {
            // Determine current drive state.
            if (!motorFound && channel->GetAttachedDevice() == DeviceType::Motor) {\
                currentSpeedPerc = channel->GetCurrentPwrPerc();
                targetSpeedPerc = channel->GetTargetPwrPerc();
                motorFound = true;
            }
            // Update current channel pwr, if needed.
            channel->UpdateCurrentPwrPerc();

            switch (channel->GetChannel()) {
            case BLEHubChannel::A:
                channelAForward = channel->IsDrivingForward();
                channelAPwr = 'A';
                break;
            case BLEHubChannel::B:
                channelBForward = channel->IsDrivingForward();
                channelBPwr = 'B';
                break;
            case BLEHubChannel::C:
                channelCForward = channel->IsDrivingForward();
                channelCPwr = 'C';
                break;
            case BLEHubChannel::D:
                channelDForward = channel->IsDrivingForward();
                channelDPwr = 'D';
                break;
            }
        }

        // Send drive command.
        if (currentSpeedPerc != targetSpeedPerc) {
            log4MC::vlogf(LOG_INFO, "DUM : Written value c=%d, t=%d",currentSpeedPerc, targetSpeedPerc); // should only be shown when changed
        }
        // Wait half the watchdog timeout (converted from s/10 to s/1000).
        // vTaskDelay(_watchdogTimeOutInTensOfSeconds * 50 / portTICK_PERIOD_MS);
        // Wait 50 milliseconds.
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}

int16_t DUMMYHub::MapPwrPercToRaw(int pwrPerc)
{
    // Map absolute speed (no matter the direction) to raw channel speed.
    return map(abs(pwrPerc), 0, 100, 0, DUMMY_MAX_CHANNEL_SPEED);
}

void DUMMYHub::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    // TODO: Ignored for now...
}
