#pragma once

#include <Arduino.h>

#include "BLEHub.h"

#define PU_REMOTECONTROL_SERVICE_UUID "00001623-1212-efde-1623-785feabcd123"
#define PU_REMOTECONTROL_CHARACTERISTIC_UUID "00001624-1212-efde-1623-785feabcd123"

#define PU_MIN_SPEED_FORWARD 0
#define PU_MAX_SPEED_FORWARD 126
#define PU_MIN_SPEED_REVERSE 255
#define PU_MAX_SPEED_REVERSE 128

class PUHub : public BLEHub
{
public:
    PUHub(std::string deviceName, std::string deviceAddress, std::vector<ChannelConfiguration> channels[], int16_t lightPerc = 100, bool autoLightsOnEnabled = false, bool enabled = true);
    bool Connect(const uint8_t watchdogTimeOutInTensOfSeconds);
    void DriveTaskLoop();
    int16_t GetMinRawChannelSpeed();
    int16_t GetMaxRawChannelSpeed();
    int16_t MapSpeedPercToRaw(int speedPerc);

private:
    bool setWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds);
};