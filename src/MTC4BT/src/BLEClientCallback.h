#pragma once

#include "NimBLEDevice.h"
#include "BLEHub.h"

class BLEClientCallback : public NimBLEClientCallbacks
{
public:
  BLEClientCallback(BLEHub *hub);

private:
  void onConnect(NimBLEClient *pclient);
  void onDisconnect(NimBLEClient *pclient);

  BLEHub *_hub;
};