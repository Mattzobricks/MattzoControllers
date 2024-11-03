#pragma once

#include "BLEHubScanner.h"
#include "BLELocomotive.h"
#include "MController.h"
#include "MTC4BTConfiguration.h"

class MTC4BTController : public MController
{
  public:
    // MTC4BTController constructor.
    MTC4BTController();

    // Locomotives under control of this controller.
    std::vector<BLELocomotive *> Locomotives;

    // Controller setup.
    void Setup(MTC4BTConfiguration *config);

    // Start the scanner
    void SetupScanner();

    // Controller loop.
    void Loop();

    // Returns a boolean value indicating whether this controller controls the loco with the given address.
    bool HasLocomotive(uint address);

    // Handles sys emergency brake command for all locos (under control of this controller).
    void HandleSys(const bool ebrake);

    // Handles the given loco command (if loco is under control if this controller).
    void HandleLc(int locoAddress, int speed, int minSpeed, int maxSpeed, char *mode, bool dirForward);

    // Handles the given trigger (if loco is under control of this controller).
    void HandleTrigger(int locoAddress, MCTriggerSource source, std::string eventType, std::string eventId, std::string value);

  private:
    // Discovers new BLE devices.
    static void discoveryLoop(void *parm);

    // Initializes locomotives with the given configuration.
    void initLocomotives(std::vector<BLELocomotiveConfiguration *> locoConfigs);

    // Returns the locomotive with the given address.
    BLELocomotive *getLocomotive(uint address);

    // Reference to the configuration of this controller.
    MTC4BTConfiguration *_config;

    // Reference to the BLE Hub scanner used by this controller.
    BLEHubScanner *_hubScanner;
};