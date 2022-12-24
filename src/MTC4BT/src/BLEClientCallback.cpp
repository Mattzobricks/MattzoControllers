#include "BLEClientCallback.h"
#include "log4MC.h"

BLEClientCallback::BLEClientCallback(BLEHub *hub) : NimBLEClientCallbacks()
{
    _hub = hub;
}

void BLEClientCallback::onConnect(NimBLEClient *client)
{
    if (client->getPeerAddress().equals(*_hub->_config->DeviceAddress)) {
        log4MC::vlogf(LOG_INFO, "BLE : Connected to hub '%s'.", client->getPeerAddress().toString().c_str());

        /** Set initial connection parameters:
         * Timeout should be a multiple of the interval, minimum is 100ms.
         * Min interval: 64 * 1.25ms = 80ms, Max interval: 64 * 1.25ms = 80ms, 0 latency, 56 * 10ms = 560ms timeout
         */
        client->updateConnParams(64, 64, 0, 56);

        _hub->connected();
    }
}

void BLEClientCallback::onDisconnect(NimBLEClient *client)
{
    if (client->getPeerAddress().equals(*_hub->_config->DeviceAddress)) {
        log4MC::vlogf(LOG_ERR, "BLE : Disconnected from hub '%s'.", _hub->_config->DeviceAddress->toString().c_str());

        _hub->_isDiscovered = false;
        _hub->disconnected();

        if (_hub->_driveTaskHandle != NULL)
        {
            vTaskDelete(_hub->_driveTaskHandle);
        }
    }
}