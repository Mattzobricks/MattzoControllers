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

void BLERemote::initHubs()
{
    for (BLEHubConfiguration *hubConfig : _config->_hubs) {
        BLEHub *hub;

        switch (hubConfig->HubType) {
        case BLEHubType::PUController:
            hub = new PURemote(hubConfig);
            break;
        }

        if (hub) {
            hub->SetConnectCallback([this](bool connected) -> void { handleConnectCallback(connected); });
            Hubs.push_back(hub);
        }
    }

    // log4MC::vlogf(LOG_INFO, "Loco: %s hub config initialized.", _config->_name.c_str());
}