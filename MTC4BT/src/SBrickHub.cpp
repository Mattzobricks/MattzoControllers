#include <Arduino.h>

#include "BLEClientCallback.h"
#include "SBrickHub.h"

#define MAX_SBRICK_CHANNEL_COUNT 4

static BLEUUID remoteControlServiceUUID(SBRICK_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID);

const int8_t CMD_BRAKE = 0;
const int8_t CMD_DRIVE = 1;
const int8_t CMD_SET_WATCHDOG_TIMEOUT = 13;
const int8_t CMD_GET_WATCHDOG_TIMEOUT = 14;
const int8_t CMD_BREAK_WITH_PM = 19;
const int8_t CMD_GET_CHANNEL_STATUS = 34;
const int16_t SBRICK_MAX_CHANNEL_SPEED = 254;
const int16_t SBRICK_MIN_CHANNEL_SPEED = -254;

SBrickHub::SBrickHub(std::string deviceName, std::string deviceAddress, std::vector<ChannelConfiguration> channels[], int16_t lightPerc, bool autoLightsOnEnabled, bool enabled)
    : BLEHub(deviceName, deviceAddress, channels, lightPerc, autoLightsOnEnabled, enabled)
{
}

bool SBrickHub::Connect(const uint8_t watchdogTimeOutInTensOfSeconds)
{
    Serial.print("[" + String(xPortGetCoreID()) + "] BLE : Connecting to ");
    Serial.println(_address->toString().c_str());

    if (_hub == nullptr)
    {
        _hub = NimBLEDevice::createClient(*_address);
    }

    if (_clientCallback == nullptr)
    {
        _clientCallback = new BLEClientCallback(this);
    }
    _hub->setClientCallbacks(_clientCallback);

    /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
   *  These settings are safe for 3 clients to connect reliably, can go faster if you have less connections. 
   *  Timeout should be a multiple of the interval, minimum is 100ms.
   *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
   */
    //_sbrick->setConnectionParams(12, 12, 0, 51);

    /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
    _hub->setConnectTimeout(ConnectDelayInSeconds);

    // Connect to the remote BLE Server.
    if (!_hub->connect(false))
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
        _hub->disconnect();
        return false;
    }

    //if(_remoteControlCharacteristic->canNotify()) {
    //  _remoteControlCharacteristic->registerForNotify(notifyCallback);
    //}

    // Start drive task loop.
    return StartDriveTask();
}

void SBrickHub::DriveTaskLoop()
{
    for (;;)
    {
        if (!_ebreak)
        {
            // Update current channel speeds, if needed.
            for (int channel = 0; channel < _channelControllers.size(); channel++)
            {
                _channelControllers.at(channel)->UpdateCurrentSpeedPerc();

                // int16_t rawspd = MapSpeedPercToRaw(_channelControllers.at(channel)->GetCurrentSpeedPerc());
                // if (rawspd != 0)
                // {
                //     Serial.print(channel);
                //     Serial.print(": rawspd=");
                //     Serial.println(rawspd);
                // }
            }

            // Construct drive command.
            uint8_t byteCmd[13] = {
                CMD_DRIVE,
                // getDriveCommand(HubChannel::A),
                HubChannel::A,
                channelIsDrivingForward(HubChannel::A),
                getRawChannelSpeed(HubChannel::A),
                HubChannel::B,
                channelIsDrivingForward(HubChannel::B),
                getRawChannelSpeed(HubChannel::B),
                HubChannel::C,
                channelIsDrivingForward(HubChannel::C),
                getRawChannelSpeed(HubChannel::C),
                HubChannel::D,
                channelIsDrivingForward(HubChannel::D),
                getRawChannelSpeed(HubChannel::D)};

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

int16_t SBrickHub::MapSpeedPercToRaw(int speedPerc)
{
    // Map absolute speed (no matter the direction) to raw channel speed.
    return map(abs(speedPerc), 0, 100, 0, SBRICK_MAX_CHANNEL_SPEED);
}

bool SBrickHub::setWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
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

std::array<uint8_t, 3> SBrickHub::getDriveCommand(HubChannel channel)
{
    ChannelController *controller = findControllerByChannel(channel);

    std::array<uint8_t, 3> cmd;
    cmd[0] = channel;
    cmd[1] = controller ? controller->IsDrivingForward() : false;
    cmd[2] = controller ? MapSpeedPercToRaw(controller->GetCurrentSpeedPerc()) : 0;

    return cmd;
}

bool SBrickHub::channelIsDrivingForward(HubChannel channel)
{
    ChannelController *controller = findControllerByChannel(channel);
    return controller ? controller->IsDrivingForward() : false;
}

uint8_t SBrickHub::getRawChannelSpeed(HubChannel channel)
{
    ChannelController *controller = findControllerByChannel(channel);
    return controller ? MapSpeedPercToRaw(controller->GetCurrentSpeedPerc()) : 0;
}