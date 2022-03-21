#include <Arduino.h>

#include "PUHub.h"
#include "log4MC.h"

#define MAX_PUHUB_CHANNEL_COUNT 2

static BLEUUID remoteControlServiceUUID(PU_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(PU_REMOTECONTROL_CHARACTERISTIC_UUID);

PUHub::PUHub(BLEHubConfiguration *config)
    : BLEHub(config)
{
    _hubLedPort = 0;
}

bool PUHub::SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
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

    log4MC::vlogf(LOG_INFO, "BLE : Watchdog timeout not set for PU hubs");

    return true;
}

void PUHub::DriveTaskLoop()
{
    for (;;)
    {
        bool motorFound = false;
        int16_t currentSpeedPerc = 0;
        int16_t targetSpeedPerc = 0;

        for (BLEHubChannelController *controller : _channelControllers)
        {
            // Determine current drive state.
            if (!motorFound && controller->GetAttachedDevice() == DeviceType::Motor)
            {
                currentSpeedPerc = controller->GetCurrentPwrPerc();
                targetSpeedPerc = controller->GetTargetPwrPerc();
                motorFound = true;
            }

            // Update current channel pwr.
            controller->UpdateCurrentPwrPerc();

            // Construct drive command.
            byte channelPwr = getRawChannelPwrForController(controller);
            byte setMotorCommand[8] = {0x81, (byte)controller->GetChannel(), 0x11, 0x51, 0x00, channelPwr};
            writeValue(setMotorCommand, 6);
        }

        // Set integrated powered up hub light according to current drive state.
        if (currentSpeedPerc != targetSpeedPerc)
        {
            // accelerating / braking
            setLedColor(PUHubLedColor::YELLOW);
        }
        else if (currentSpeedPerc != 0)
        {
            // travelling at target speed
            setLedColor(PUHubLedColor::GREEN);
        }
        else
        {
            // stopped
            setLedColor(PUHubLedColor::RED);
        }

        // Wait half the watchdog timeout (converted from s/10 to s/1000).
        // vTaskDelay(_watchdogTimeOutInTensOfSeconds * 50 / portTICK_PERIOD_MS);

        // Wait 50 milliseconds.
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

int16_t PUHub::MapPwrPercToRaw(int pwrPerc)
{
    if (pwrPerc == 0)
    {
        return 0; // 0 = float, 127 = stop motor
    }

    if (pwrPerc > 0)
    {
        return map(pwrPerc, 0, 100, PU_MIN_SPEED_FORWARD, PU_MAX_SPEED_FORWARD);
    }

    return map(abs(pwrPerc), 0, 100, PU_MIN_SPEED_REVERSE, PU_MAX_SPEED_REVERSE);
}

void PUHub::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    switch (pData[2])
    {
    // case (byte)MessageType::HUB_PROPERTIES:
    // {
    //     parseDeviceInfo(pData);
    //     break;
    // }
    case (byte)MessageType::HUB_ATTACHED_IO:
    {
        parsePortMessage(pData);
        break;
    }
        // case (byte)MessageType::PORT_VALUE_SINGLE:
        // {
        //     parseSensorMessage(pData);
        //     break;
        // }
        // case (byte)MessageType::PORT_OUTPUT_COMMAND_FEEDBACK:
        // {
        //     parsePortAction(pData);
        //     break;
        // }
    }
}

/**
 * @brief Parse the incoming characteristic notification for a Port Message
 * @param [in] pData The pointer to the received data
 */
void PUHub::parsePortMessage(uint8_t *pData)
{
    byte port = pData[3];
    bool isConnected = (pData[4] == 1 || pData[4] == 2) ? true : false;
    if (isConnected)
    {
        // log4MC::vlogf(LOG_INFO, "port %x is connected with device %x", port, pData[5]);
        if (pData[5] == 0x0017)
        {
            _hubLedPort = port;
            log4MC::vlogf(LOG_INFO, "PU  : Found integrated RGB light at port %x", port);
        }
    }
}

void PUHub::setLedColor(PUHubLedColor color)
{
    if (_hubLedPort == 0)
    {
        return;
    }

    byte setColorMode[8] = {0x41, _hubLedPort, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    writeValue(setColorMode, 8);

    byte setColor[6] = {0x81, _hubLedPort, 0x11, 0x51, 0x00, color};
    writeValue(setColor, 6);
}

void PUHub::writeValue(byte command[], int size)
{
    byte byteCmd[size + 2] = {(byte)(size + 2), 0x00};
    memcpy(byteCmd + 2, command, size);

    // Send drive command.
    if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), false))
    {
        log4MC::vlogf(LOG_ERR, "BLE : Drive failed (%s). Unabled to write to PU characteristic.", GetAddress().toString().c_str());
    }
}