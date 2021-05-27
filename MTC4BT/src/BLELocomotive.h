#pragma once

#include <Arduino.h>

#include "BLEHub.h"
#include "BLELocomotiveConfiguration.h"
#include "MCLedBase.h"

#define AUTO_LIGHTS_ENABLED true

class BLELocomotive
{
public:
    BLELocomotive(BLELocomotiveConfiguration *config, MController *controller);

    // List of references to hubs inside this loco.
    std::vector<BLEHub *> Hubs;

    // Returns a boolean value indicating whether this loco is enabled (in use).
    bool IsEnabled();

    // Returns a boolean value indicating whether we are connected to all BLE hubs.
    bool AllHubsConnected();

    // Sets the given target speed for all motor channels for all hubs.
    void Drive(const int16_t minSpeed, const int16_t speed);

    // Returns a list of functions that need to be handled.
    std::vector<Fn *> GetFn(MCFunction func);

    // Turns the specified function on/off.
    void HandleFn(Fn *fn, const bool on);

    // If true, immediately sets the current speed for all channels on all hubs to zero and makes all loco lights blink.
    // If false, releases the emergency brake, returning the loco to normal operations.
    void EmergencyBrake(const bool enabled);

    // Returns the loco name.
    std::string GetLocoName();

    // Returns the loco address.
    uint GetLocoAddress();

    // Returns the number of hubs in the loco.
    uint GetHubCount();

    // Returns the hub at the given index.
    BLEHub *GetHub(uint index);

    // Returns a boolean value indicating whether the lights should automatically turn on when the loco starts driving.
    bool GetAutoLightsEnabled();

private:
    // Initialized the leds inside this loco.
    void initLights();

    // Initialized the hubs inside this loco.
    void initHubs();

    // Returns a reference to a hub by its address.
    BLEHub *getHubByAddress(std::string address);

    // Returns a list of references to functions of the given type configured for this loco.
    std::vector<Fn *> getFunctions(MCFunction f);

    // List of references to leds inside this loco.
    std::vector<MCLedBase *> _espLeds;

    // Reference to the configuration of this loco.
    BLELocomotiveConfiguration *_config;

    // Reference to the controller controling this loco.
    MController *_controller;
};