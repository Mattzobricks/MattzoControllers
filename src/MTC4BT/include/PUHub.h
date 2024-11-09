#pragma once

#include <Arduino.h>

#include "BLEHub.h"

#define PU_REMOTECONTROL_SERVICE_UUID "00001623-1212-efde-1623-785feabcd123"
#define PU_REMOTECONTROL_CHARACTERISTIC_UUID "00001624-1212-efde-1623-785feabcd123"

#define PU_MIN_SPEED_FORWARD 0
#define PU_MAX_SPEED_FORWARD 126
#define PU_MAX_SPEED_REVERSE 128
#define PU_MIN_SPEED_REVERSE 255

class PUHub : public BLEHub
{
  public:
    PUHub(BLEHubConfiguration *config);
    bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds);
    void DriveTaskLoop();
    int16_t MapPwrPercToRaw(int pwrPerc);

    /**
     * @brief Callback function for notifications of a specific characteristic
     * @param [in] pBLERemoteCharacteristic The pointer to the characteristic
     * @param [in] pData The pointer to the received data
     * @param [in] length The length of the data array
     * @param [in] isNotify
     */
    void NotifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);

  protected:
    byte _hubLedPort;
    void parsePortAction(uint8_t *pData, size_t length);
    void parsePortMessage(uint8_t *pData);
    void setLedColor(HubLedColor color);
    void setLedHSVColor(int hue, double saturation, double value);
    void setLedRGBColor(char red, char green, char blue);
    void writeValue(byte command[], int size);
};