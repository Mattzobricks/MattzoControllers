#pragma once

#include "NimBLEDevice.h"
#include "SBrickHubClient.h"

class SBrickClientCallback : public NimBLEClientCallbacks
{
public:
  SBrickClientCallback(SBrickHubClient *sbrick);

private:
  void onConnect(NimBLEClient *pclient);
  void onDisconnect(NimBLEClient *pclient);

  SBrickHubClient *_psbrick;
};