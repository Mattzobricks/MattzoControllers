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
	void onMTUChange(NimBLEClient* pClient, uint16_t mtu);

	BLEHub *_hub;
};