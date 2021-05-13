#include "NimBLEDevice.h"
#include "BLEHub.h"
#include "BLEClientCallback.h"
#include "log4MC.h"

BLEClientCallback::BLEClientCallback(BLEHub *hub) : NimBLEClientCallbacks()
{
    _hub = hub;
}

void BLEClientCallback::onConnect(NimBLEClient *client)
{
    if (client->getPeerAddress().equals(*_hub->_config->DeviceAddress))
    {
        log4MC::vlogf(LOG_INFO, "BLE : Connected to hub '%s'.", client->getPeerAddress().toString().c_str());

        _hub->_isConnected = true;
    }
}

void BLEClientCallback::onDisconnect(NimBLEClient *client)
{
    if (client->getPeerAddress().equals(*_hub->_config->DeviceAddress))
    {
        log4MC::vlogf(LOG_ERR, "BLE : Disconnected from hub '%s'.", _hub->_config->DeviceAddress->toString().c_str());

        _hub->_isDiscovered = false;
        _hub->_isConnected = false;
        if (_hub->_driveTaskHandle != NULL)
        {
            vTaskDelete(_hub->_driveTaskHandle);
        }

        client->disconnect();
    }
}