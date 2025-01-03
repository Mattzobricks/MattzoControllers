#include <Arduino.h>

#include "BuWizz2Hub.h"
#include "DriverTaskDelay.h"
#include "log4MC.h"

#define MAX_BuWizz2Hub_CHANNEL_COUNT 4

static BLEUUID remoteControlServiceUUID(BUWIZZ2_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(BUWIZZ2_REMOTECONTROL_CHARACTERISTIC_UUID);

BuWizz2Hub::BuWizz2Hub(BLEHubConfiguration *config)
    : BLEHub(config)
{
    batteryVoltage = 0.0;
    status = 0;
    powerLevel = 0;
    defaultPowerLevel = config->powerLevel; // 2; // Normal
}

bool BuWizz2Hub::SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds)
{
    _watchdogTimeOutInTensOfSeconds = watchdogTimeOutInTensOfSeconds;

    if (!attachCharacteristic(remoteControlServiceUUID, remoteControlCharacteristicUUID)) {
        log4MC::error("BLE : BuWizz2 : Unable to attach to remote control service.");
        return false;
    }

    if (!_remoteControlCharacteristic->canWrite()) {
        log4MC::error("BLE : BuWizz2 : Remote control characteristic doesn't allow writing.");
        return false;
    }

    log4MC::vlogf(LOG_INFO, "BLE : BuWizz2 : Watchdog timeout not set");

    return true;
}

/*
0x10 Set motor data
Transfers motor data to the device.
Byte         | Function / value
0 (command)  | 0x10 - Set motor data
1-4          | 0x10 - Set motor data
             | Motor data (signed 8-bit value for each motor output)
             | 0x81 (-127): Full backwards
             | 0x00 (0): Stop
             | 0x7F (127): Full forwards
5            | Brake flags - bit mapped to bits 3-0 (1 bit per each motor, bit 0 for first motor, bit 3 for the last)
             | If brake flag is set for a target motor, slow-decay control mode will be used
             | (shortcircuiting the motor armature over the inactive phase).
             | If flag is not set, the corresponding motor will be controlled in fast-decay control
             | method (coasting the motor during the inactive phase).
No response is generated

<Q> only send following command at connction, because it is in the config?
0x11 Set power level
Changes the power level.
Byte        | Function / value
0 (command) | 0x11 - Set power level
1           | Power level index
            | Value | Function
            | 0     | Power is disabled (default value after start or BLE disconnect)
            | 1     | Slow
            | 2     | Normal
            | 3     | Fast
            | 4     | LDCRS
No response is sent.
*/
void BuWizz2Hub::DriveTaskLoop()
{
    uint8_t channel1Pwr = 0;
    uint8_t channel2Pwr = 0;
    uint8_t channel3Pwr = 0;
    uint8_t channel4Pwr = 0;

    uint8_t oldChannel1Pwr = 1;
    uint8_t oldChannel2Pwr = 1;
    uint8_t oldChannel3Pwr = 1;
    uint8_t oldChannel4Pwr = 1;

    for (;;) {
        if (powerLevel != defaultPowerLevel) { // powerlevel is updated by the NotifyCall back and is always up to date
            // set powerlevel of the device
            uint8_t byteCmd[2] = {
                SET_POWER_LEVEL,
                defaultPowerLevel};
            log4MC::vlogf(LOG_DEBUG, "Set power level %02x", defaultPowerLevel);
            if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), true)) {
                log4MC::vlogf(LOG_ERR, "BUWIZZ2 : Drive failed. Unable to write to BUWIZZ2 characteristic.");
            }
        }

        // values for all 4 ports
        for (BLEHubChannelController *controller : _channelControllers) {
            controller->UpdateCurrentPwrPerc();

            switch (controller->GetHubChannel()) {
            case BLEHubChannel::A: // 1
                channel1Pwr = getRawChannelPwrForController(controller);
                break;
            case BLEHubChannel::B: // 2
                channel2Pwr = getRawChannelPwrForController(controller);
                break;
            case BLEHubChannel::C: // 3
                channel3Pwr = getRawChannelPwrForController(controller);
                break;
            case BLEHubChannel::D: // 4
                channel4Pwr = getRawChannelPwrForController(controller);
                break;
            case OnboardLED: // not available on the BuWizz 2.0
                break;
            }
        }
        if ((channel1Pwr != oldChannel1Pwr) ||
            (channel2Pwr != oldChannel2Pwr) ||
            (channel3Pwr != oldChannel3Pwr) ||
            (channel4Pwr != oldChannel4Pwr)) {

            oldChannel1Pwr = channel1Pwr;
            oldChannel2Pwr = channel2Pwr;
            oldChannel3Pwr = channel3Pwr;
            oldChannel4Pwr = channel4Pwr;
            // Construct one drive command for all channels.
            uint8_t
                byteCmd[6] = {
                    SET_MOTOR_DATA,
                    channel1Pwr,
                    channel2Pwr,
                    channel3Pwr,
                    channel4Pwr,
                    0}; // break flag

            // log4MC::vlogf(LOG_DEBUG, "Motor 1,2,3,4 %02x %02x %02x %02x", channel1Pwr, channel2Pwr, channel3Pwr, channel4Pwr);
            if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), true)) {
                log4MC::vlogf(LOG_ERR, "BUWIZZ2 : Drive failed. Unable to write to BUWIZZ2 characteristic.");
            }
        }
        // Wait half the watchdog timeout (converted from s/10 to s/1000).
        // vTaskDelay(_watchdogTimeOutInTensOfSeconds * 50 / portTICK_PERIOD_MS);

        // Wait 50 milliseconds.
        vTaskDelay(DRIVERTASKDELAY / portTICK_PERIOD_MS);
    }
}

/*
 0x81 (-127): Full backwards
 0x00 (0): Stop
 0x7F (127): Full forwards

 map(value, fromLow, fromHigh, toLow, toHigh)
*/
int16_t BuWizz2Hub::MapPwrPercToRaw(int pwrPerc)
{
    int16_t retval;
    // log4MC::vlogf(LOG_DEBUG, " BuWizz2: positive map: %d", pwrPerc);
    if (pwrPerc == 0) {
        return 0; // 0 = float, 127 = stop motor
    }

    if (pwrPerc > 0) {
        retval = map(pwrPerc, 0, 100, BUWIZZ2_MIN_SPEED_FORWARD, BUWIZZ2_MAX_SPEED_FORWARD);
        // log4MC::vlogf(LOG_DEBUG, " positive map: %4x", retval);
        return retval;
    }
    retval = map(abs(pwrPerc), 0, 100, BUWIZZ2_MIN_SPEED_REVERSE, BUWIZZ2_MAX_SPEED_REVERSE);
    // log4MC::vlogf(LOG_DEBUG, " negative map: %4x", retval);
    return retval;
}

void BuWizz2Hub::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    switch (pData[0]) {
    case (byte)BuWizzMessageType::BUW2_DEVICE_STATUS:
        parseDeviceSatusMessage(pData, length);
        // log4MC::vlogf(LOG_DEBUG,"BatteryVoltage: %f",batteryVoltage);
        break;
#ifdef DEBUGNOTIFYBUWIZZ2
    default:
        dumpPData(pData, length);
#endif
    }
}

/*
[0169] [0] 0000 00 18 5c 00 00 00 00 00 00 19 25 00 fc ff b9 fa  ..\.......%.....
[0170] [0] 0010 00 00 00 00                                      '....'

00 = cmd 00 device status report
18 = 11000 -> battery full // 78 = 1111000
5c = battery voltage = 92*0.01+3 = 3.92 Volt
00 = output voltage motor = 0
00 00 00 00 = motor currents = 0
00 current power level = 0 = disable
19 current temp = 25 deg C
25 00 Accelerometer x-axis value (left-aligned 12-bit signed value, 12 mg/digit)
fc ff Accelerometer y-axis value
b9 fa Accelerometer z-axis value

Byte        | Function / value
0 (command) |0x00
1           | Status flags - bit mapped to the following functions:
            | Bit | Function
            | 7   | unused
            | 6   | USB connection status (1 - cable connected)
            | 5   | Battery charging status (1 - battery is charging, 0 - battery is full or not charging)
            | 3-4 | Battery level status (0 - empty, motors disabled; 1 - low; 2 - medium; 3 - full)
            | 2-1 | unused
            | 0   | error (overcurrent, overtemperature...)
2           | Battery voltage (3 V + value * 0,01 V) - range 3,00 V - 4,27 V
            | Example: 0x00 => 3,00 V, 0x7F => 4,27 V
3           | Output (motor) voltage (4 V + value * 0,05 V) - range 4,00 V - 16,75 V
4-7         | Motor currents, 8-bit value for each motor output (value * 0,033 A) - range 0 - 8,5 A
8           | Current power level
            | Value | Function
            | 0     | Power is disabled (default value after start or BLE disconnect)
            | 1     | Slow
            | 2     | Normal
            | 3     | Fast
            | 4     | LDCRS
9           | Microcontroller temperature (value in °C)
10-11       | Accelerometer x-axis value (left-aligned 12-bit signed value, 12 mg/digit)
12-13       | Accelerometer y-axis value (left-aligned 12-bit signed value, 12 mg/digit)
14-15       | Accelerometer z-axis value (left-aligned 12-bit signed value, 12 mg/digit)
*/
/**
 * @brief Parse the incoming characteristic notification for a Port Message
 * @param [in] pData The pointer to the received data
 */
void BuWizz2Hub::parseDeviceSatusMessage(uint8_t *pData, size_t length)
{

    uint8_t digitalVbat = pData[2];
    batteryVoltage = 3.0 + digitalVbat * 0.01;
    status = pData[1];
    powerLevel = pData[8];
}

void BuWizz2Hub::setPowerLevel(uint8_t newLevel)
{
    defaultPowerLevel = newLevel;
}
