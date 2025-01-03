#pragma once
/*
* BLEbaseDevice is the base class for BLELocomotive and BLERemote,
* all the methods and variables bot have in common are here.
*/


#include <Arduino.h>

#include "BLEHub.h"
#include "MController.h"

class BLEbaseDevice
{
  public:
    BLEbaseDevice(MController *controller);

    // List of references to hubs inside this loco.
    std::vector<BLEHub *> Hubs;

    // Returns always true for the base device
    bool AllHubsConnected();

    // Makes all channels on all hubs with lights attached blink for the given duration.
    void BlinkLights(int durationInMs);

    // Sets a new color for hubs that have an onboard LED.
    void SetHubLedColor(HubLedColor color);

    // Returns the number of hubs in the loco.
    uint GetHubCount();

    // Returns the hub at the given index.
    BLEHub *GetHub(uint index);

  protected:
    // Initialized the hubs inside this loco.
    void initHubs();

    // Returns a reference to a hub by its address.
    BLEHub *getHubByAddress(std::string address);

    // Reference to the controller controling this loco.
    MController *_controller;
};