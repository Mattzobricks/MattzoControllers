#include "NimBLEDevice.h"
#include "BLEHub.h"
#include "BLEClientCallback.h"

BLEClientCallback::BLEClientCallback(BLEHub *hub) : NimBLEClientCallbacks()
{
    _hub = hub;
}

void BLEClientCallback::onConnect(NimBLEClient *client)
{
    if (client->getPeerAddress().equals(*_hub->_config->DeviceAddress))
    {
        Serial.print("[" + String(xPortGetCoreID()) + "] BLE : onConnect: ");
        Serial.println(client->getPeerAddress().toString().c_str());
        _hub->_isConnected = true;
    }
}

void BLEClientCallback::onDisconnect(NimBLEClient *client)
{
    if (client->getPeerAddress().equals(*_hub->_config->DeviceAddress))
    {
        Serial.print("[" + String(xPortGetCoreID()) + "] BLE : onDisconnect: ");
        Serial.println(_hub->_config->DeviceAddress->toString().c_str());
        _hub->_isDiscovered = false;
        _hub->_isConnected = false;
        // TODO: _hub->Drive(0, 0);
        if (_hub->_driveTaskHandle != NULL)
        {
            vTaskDelete(_hub->_driveTaskHandle);
        }

        client->disconnect();
    }
}
