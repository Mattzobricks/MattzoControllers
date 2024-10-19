#include <Arduino.h>

#include "BuWizz2Hub.h"
#include "log4MC.h"

#define MAX_BuWizz2Hub_CHANNEL_COUNT 4

static BLEUUID remoteControlServiceUUID(BUWIZZ2_REMOTECONTROL_SERVICE_UUID);
static BLEUUID remoteControlCharacteristicUUID(BUWIZZ2_REMOTECONTROL_CHARACTERISTIC_UUID);

BuWizz2Hub::BuWizz2Hub(BLEHubConfiguration *config)
    : BLEHub(config)
{
    _hubLedPort = 0;
    batteryVoltage = 0.0;
    status = 0;
    powerLevel = 0;
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

void BuWizz2Hub::DriveTaskLoop()
{
    for (;;) {
        bool motorFound = false;
        int16_t currentSpeedPerc = 0;
        int16_t targetSpeedPerc = 0;

        for (BLEHubChannelController *controller : _channelControllers) {
            // Determine current drive state.
            if (!motorFound && controller->GetAttachedDevice() == DeviceType::Motor) {
                currentSpeedPerc = controller->GetCurrentPwrPerc();
                targetSpeedPerc = controller->GetTargetPwrPerc();
                motorFound = true;
            }

            if (controller->GetHubChannel() == BLEHubChannel::OnboardLED) {
                // Update onboard LED channel state.
                setLedColor(getRawLedColorForController(controller));
            } else {
                // Update current channel pwr.
                controller->UpdateCurrentPwrPerc();

                // Construct drive command.
                byte channelPwr = getRawChannelPwrForController(controller);
                byte setMotorCommand[6] = {0x81, (byte)controller->GetHubChannel(), 0x11, 0x51, 0x00, channelPwr};
                writeValue(setMotorCommand, 6);
            }
        }

        // Wait half the watchdog timeout (converted from s/10 to s/1000).
        // vTaskDelay(_watchdogTimeOutInTensOfSeconds * 50 / portTICK_PERIOD_MS);

        // Wait 50 milliseconds.
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

int16_t BuWizz2Hub::MapPwrPercToRaw(int pwrPerc)
{
    if (pwrPerc == 0) {
        return 0; // 0 = float, 127 = stop motor
    }

    if (pwrPerc > 0) {
        return map(pwrPerc, 0, 100, PU_MIN_SPEED_FORWARD, PU_MAX_SPEED_FORWARD);
    }

    return map(abs(pwrPerc), 0, 100, PU_MIN_SPEED_REVERSE, PU_MAX_SPEED_REVERSE);
}

void BuWizz2Hub::NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    switch (pData[0]) {
    // case (byte)MessageType::HUB_PROPERTIES:
    // {
    //     parseDeviceInfo(pData);
    //     break;
    // }
    case (byte)MessageType::BUW2_DEVICE_STATUS:
        parseDeviceSatusMessage(pData,length);
        // log4MC::vlogf(LOG_DEBUG,"BatteryVoltage: %f",batteryVoltage);
        break;

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
    default:
#ifdef DEBUGNOTIFYBUWIZZ2
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

1
2
3
4-7
8
9
10-11
12-13
14-15
7
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
    batteryVoltage = 3.0+ digitalVbat*0.01;
    status = pData[1];
    powerLevel = pData[8];
}

/**
 * @brief Set the color of the HUB LED with predefined colors
 * @param [in] color one of the available hub colors
 */
void BuWizz2Hub::setLedColor(HubLedColor color)
{
    if (_hubLedPort == 0) {
        return;
    }

    byte setColorMode[8] = {0x41, _hubLedPort, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    writeValue(setColorMode, 8);

    byte setColor[6] = {0x81, _hubLedPort, 0x11, 0x51, 0x00, color};
    writeValue(setColor, 6);
}

/**
 * @brief Set the color of the HUB LED with HSV values
 * @param [in] hue 0..360
 * @param [in] saturation 0..1
 * @param [in] value 0..1
 */
void BuWizz2Hub::setLedHSVColor(int hue, double saturation, double value)
{
    hue = hue % 360; // map hue to 0..360
    double huePart = hue / 60.0;
    double fract = huePart - floor(huePart);

    double p = value * (1. - saturation);
    double q = value * (1. - saturation * fract);
    double t = value * (1. - saturation * (1. - fract));

    if (huePart >= 0.0 && huePart < 1.0) {
        setLedRGBColor((char)(value * 255), (char)(t * 255), (char)(p * 255));
    } else if (huePart >= 1.0 && huePart < 2.0) {
        setLedRGBColor((char)(q * 255), (char)(value * 255), (char)(p * 255));
    } else if (huePart >= 2.0 && huePart < 3.0) {
        setLedRGBColor((char)(p * 255), (char)(value * 255), (char)(t * 255));
    } else if (huePart >= 3.0 && huePart < 4.0) {
        setLedRGBColor((char)(p * 255), (char)(q * 255), (char)(value * 255));
    } else if (huePart >= 4.0 && huePart < 5.0) {
        setLedRGBColor((char)(t * 255), (char)(p * 255), (char)(value * 255));
    } else if (huePart >= 5.0 && huePart < 6.0) {
        setLedRGBColor((char)(value * 255), (char)(p * 255), (char)(q * 255));
    } else {
        setLedRGBColor(0, 0, 0);
    }
}

/**
 * @brief Set the color of the HUB LED with RGB values
 * @param [in] red 0..255
 * @param [in] green 0..255
 * @param [in] blue 0..255
 */
void BuWizz2Hub::setLedRGBColor(char red, char green, char blue)
{
    byte setRGBMode[8] = {0x41, _hubLedPort, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
    writeValue(setRGBMode, 8);

    byte setRGBColor[8] = {0x81, _hubLedPort, 0x11, 0x51, 0x01, red, green, blue};
    writeValue(setRGBColor, 8);
}

void BuWizz2Hub::writeValue(byte command[], int size)
{
    byte byteCmd[size + 2] = {(byte)(size + 2), 0x00};
    memcpy(byteCmd + 2, command, size);

    // Send drive command.
    if (!_remoteControlCharacteristic->writeValue(byteCmd, sizeof(byteCmd), false)) {
        log4MC::vlogf(LOG_ERR, "BLE : Drive failed (%s). Unabled to write to PU characteristic.", GetAddress().toString().c_str());
    }
}