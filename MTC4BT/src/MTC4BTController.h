#pragma once

#include "MCController.h"
#include "MTC4BTConfiguration.h"
#include "BLELocomotive.h"
#include "BLEHubScanner.h"

class MTC4BTController : public MCController
{
public:
    // MTC4BTController constructor.
    MTC4BTController();

    // Locomotives under control of this controller.
    std::vector<BLELocomotive *> Locomotives;

    // Controller setup.
    void Setup(MTC4BTConfiguration *config);

    // Controller loop.
    void Loop();

    // Returns a boolean value indicating whether this controller controls the loco with the given address.
    bool HasLocomotive(uint address);

    // Handles sys emergency brake command for all locos (under control of this controller).
    void HandleSys(const bool ebrake);

    // Handles the given loco command (if loco is under control if this controller).
    void HandleLc(int locoAddress, int speed, int minSpeed, int maxSpeed, char *mode, bool dirForward);

    // Handles the given function for the specified loco (if loco is under control of this controller).
    void HandleFn(int locoAddress, MCFunction f, const bool on);

private:
    static void discoveryLoop(void *parm);

    void initLocomotives(std::vector<BLELocomotiveConfiguration *> locoConfigs);
    BLELocomotive *getLocomotive(uint address);

    MTC4BTConfiguration *_config;
    NimBLEScan *_scanner;
    BLEHubScanner *_hubScanner;
};