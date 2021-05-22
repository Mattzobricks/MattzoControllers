#pragma once

#include "MCController.h"
#include "MTC4BTConfiguration.h"
#include "BLELocomotive.h"
#include "BLEHubScanner.h"

class MTC4BTController : public MCController
{
public:
    MTC4BTController();

    std::vector<BLELocomotive *> Locomotives;

    void Setup(MTC4BTConfiguration *config);
    void Loop();
    void HandleEmergencyBrake(const bool enabled);
    BLELocomotive *GetLocomotive(uint address);
    void HandleFn(int locoAddress, MCFunction f, const bool on);

private:
    static void discoveryLoop(void *parm);

    void initLocomotives(std::vector<BLELocomotiveConfiguration *> locoConfigs);

    MTC4BTConfiguration *_config;
    NimBLEScan *_scanner;
    BLEHubScanner *_hubScanner;
};