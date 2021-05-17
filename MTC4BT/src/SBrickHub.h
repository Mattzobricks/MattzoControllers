#pragma once

#include <Arduino.h>

#include "BLEHub.h"

#define SBRICK_REMOTECONTROL_SERVICE_UUID "4dc591b0-857c-41de-b5f1-15abda665b0c"
#define SBRICK_REMOTECONTROL_CHARACTERISTIC_UUID "02b8cbcc-0e25-4bda-8790-a15f53e6010f"

class SBrickHub : public BLEHub
{
public:
    SBrickHub(BLEHubConfiguration *config, int16_t speedStep, int16_t brakeStep);
    bool SetWatchdogTimeout(const uint8_t watchdogTimeOutInTensOfSeconds);
    void DriveTaskLoop();
    int16_t MapSpeedPercToRaw(int speedPerc);

private:
    std::array<uint8_t, 3> getDriveCommand(HubChannel channel);
    bool channelIsDrivingForward(HubChannel channel);
    uint8_t getRawChannelSpeed(HubChannel channel);
};