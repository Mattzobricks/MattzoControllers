#pragma once

#include <Arduino.h>

#include "BLEHub.h"

#define PU_REMOTECONTROL_SERVICE_UUID "00001623-1212-efde-1623-785feabcd123"
#define PU_REMOTECONTROL_CHARACTERISTIC_UUID "00001624-1212-efde-1623-785feabcd123"

#define PU_MIN_SPEED_FORWARD 0
#define PU_MAX_SPEED_FORWARD 126
#define PU_MAX_SPEED_REVERSE 128
#define PU_MIN_SPEED_REVERSE 255

enum PUHubLedColor {
    BLACK = 0,
    PINK = 1,
    PURPLE = 2,
    BLUE = 3,
    LIGHTBLUE = 4,
    CYAN = 5,
    GREEN = 6,
    YELLOW = 7,
    ORANGE = 8,
    RED = 9,
    WHITE = 10,
    NUM_COLORS,
    NONE = 255
};

enum struct PUDeviceType {
    UNKNOWNDEVICE = 0,
    SIMPLE_MEDIUM_LINEAR_MOTOR = 1,
    TRAIN_MOTOR = 2,
    LIGHT = 8,
    HUB_LED = 23,
    MEDIUM_LINEAR_MOTOR = 38,
    MOVE_HUB_MEDIUM_LINEAR_MOTOR = 39,
    TECHNIC_LARGE_LINEAR_MOTOR = 46,  // Technic Control+
    TECHNIC_XLARGE_LINEAR_MOTOR = 47, // Technic Control+
};

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

  private:
    byte _hubLedPort;

    void parsePortMessage(uint8_t *pData);
    void setLedColor(PUHubLedColor color);
    void setLedHSVColor(int hue, double saturation, double value);
    void setLedRGBColor(char red, char green, char blue);
    void writeValue(byte command[], int size);
};