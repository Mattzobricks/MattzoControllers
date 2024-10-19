#pragma once

#include <Arduino.h>

#include "BLEHub.h"

#define BUWIZZ2_REMOTECONTROL_SERVICE_UUID        "4e050000-74fb-4481-88b3-9919b1676e93"
#define BUWIZZ2_REMOTECONTROL_CHARACTERISTIC_UUID "000092d1-0000-1000-8000-00805f9b34fb"

#define PU_MIN_SPEED_FORWARD 0
#define PU_MAX_SPEED_FORWARD 126
#define PU_MAX_SPEED_REVERSE 128
#define PU_MIN_SPEED_REVERSE 255

class BuWizz2Hub : public BLEHub
{
  public:
    BuWizz2Hub(BLEHubConfiguration *config);
    bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds);
    void DriveTaskLoop();
    int16_t MapPwrPercToRaw(int pwrPerc);
    double batteryVoltage;
    uint8_t status;
    uint8_t powerLevel;

    /**
     * @brief Callback function for notifications of a specific characteristic
     * @param [in] pBLERemoteCharacteristic The pointer to the characteristic
     * @param [in] pData The pointer to the received data
     * @param [in] length The length of the data array
     * @param [in] isNotify
     */
    void NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);

  private:
    byte _hubLedPort;

    void parseDeviceSatusMessage(uint8_t *pData, size_t length);
    void setLedColor(HubLedColor color);
    void setLedHSVColor(int hue, double saturation, double value);
    void setLedRGBColor(char red, char green, char blue);
    void writeValue(byte command[], int size);
};