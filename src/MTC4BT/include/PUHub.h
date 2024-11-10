#pragma once

#include <Arduino.h>

#include "BLEHub.h"

#define PU_REMOTECONTROL_SERVICE_UUID "00001623-1212-efde-1623-785feabcd123"
#define PU_REMOTECONTROL_CHARACTERISTIC_UUID "00001624-1212-efde-1623-785feabcd123"

#define PU_MIN_SPEED_FORWARD 0
#define PU_MAX_SPEED_FORWARD 126
#define PU_MAX_SPEED_REVERSE 128
#define PU_MIN_SPEED_REVERSE 255

enum struct PUMessageType {
    HUB_PROPERTIES = 0x01,
    HUB_ACTIONS = 0x02,
    HUB_ALERTS = 0x03,
    HUB_ATTACHED_IO = 0x04,
    GENERIC_ERROR_MESSAGES = 0x05,
    HW_NETWORK_COMMANDS = 0x08,
    FW_UPDATE_GO_INTO_BOOT_MODE = 0x10,
    FW_UPDATE_LOCK_MEMORY = 0x11,
    FW_UPDATE_LOCK_STATUS_REQUEST = 0x12,
    FW_LOCK_STATUS = 0x13,
    PORT_INFORMATION_REQUEST = 0x21,
    PORT_MODE_INFORMATION_REQUEST = 0x22,
    PORT_INPUT_FORMAT_SETUP_SINGLE = 0x41,
    PORT_INPUT_FORMAT_SETUP_COMBINEDMODE = 0x42,
    PORT_INFORMATION = 0x43,
    PORT_MODE_INFORMATION = 0x44,
    PORT_VALUE_SINGLE = 0x45,
    PORT_VALUE_COMBINEDMODE = 0x46,
    PORT_INPUT_FORMAT_SINGLE = 0x47,
    PORT_INPUT_FORMAT_COMBINEDMODE = 0x48,
    VIRTUAL_PORT_SETUP = 0x61,
    PORT_OUTPUT_COMMAND = 0x81,
    PORT_OUTPUT_COMMAND_FEEDBACK = 0x82,
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

  protected:
    byte _hubLedPort;
    void parsePortAction(uint8_t *pData, size_t length);
    void parsePortMessage(uint8_t *pData);
    void setLedColor(HubLedColor color);
    void setLedHSVColor(int hue, double saturation, double value);
    void setLedRGBColor(char red, char green, char blue);
    void writeValue(byte command[], int size);
};