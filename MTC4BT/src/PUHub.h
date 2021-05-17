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
    PUHub(BLEHubConfiguration *config, int16_t speedStep, int16_t brakeStep);
    bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds);
    void DriveTaskLoop();
    int16_t MapSpeedPercToRaw(int speedPerc);
};