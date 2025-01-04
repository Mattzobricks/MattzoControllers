/*
 * BLERemote
 */

#include "BLERemote.h"
#include "MCLightController.h"
#include "MController.h"
#include "PURemote.h"
#include "log4MC.h"

BLERemote::BLERemote(BLERemoteConfiguration *config, MController *controller)
	: BLEbaseDevice(controller), _config{config}
{
	initHubs();
}

bool BLERemote::AllHubsConnected()
{
	for (BLEHub *hub : Hubs) {
		if (!hub->IsConnected()) {
			return false;
		}
	}

	return true;
}

void BLERemote::initHubs()
{
	for (BLEHubConfiguration *hubConfig : _config->_hubs) {
		BLEHub *hub;

		switch (hubConfig->HubType) {
		case BLEHubType::PUController:
			hub = new PURemote(hubConfig);
			break;
		case PU:	  // not a remote
		case SBrick:  // not a remote
		case BuWizz2: // not a remote
			break;
		}
		// Next is needed for connection scanning!
		if (hub) {
			// hub->SetConnectCallback([this](bool connected) -> void { handleConnectCallback(connected); });
			Hubs.push_back(hub);
		}
	}
}