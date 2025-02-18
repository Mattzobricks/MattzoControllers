#pragma once

#include "BLEHub.h"
#include "NimBLEDevice.h"

class BLEClientCallback : public NimBLEClientCallbacks
{
  public:
	BLEClientCallback(BLEHub *hub);

  private:
	void onConnect(NimBLEClient *pclient);
	void onDisconnect(NimBLEClient *pclient, int reason);

	BLEHub *_hub;
};