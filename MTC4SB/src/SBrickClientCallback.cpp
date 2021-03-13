#include "NimBLEDevice.h"
#include "SBrickHubClient.h"
#include "SBrickClientCallback.h"

SBrickClientCallback::SBrickClientCallback(SBrickHubClient *sbrick) : NimBLEClientCallbacks()
{
    _psbrick = sbrick;
}

void SBrickClientCallback::onConnect(NimBLEClient *pclient)
{
    if (pclient->getPeerAddress().equals(*_psbrick->_address))
    {
        Serial.print("[" + String(xPortGetCoreID()) + "] BLE : onConnect: ");
        Serial.println(pclient->getPeerAddress().toString().c_str());
        _psbrick->_isConnected = true;
    }
}

void SBrickClientCallback::onDisconnect(NimBLEClient *pclient)
{
    if (pclient->getPeerAddress().equals(*_psbrick->_address))
    {
        Serial.print("[" + String(xPortGetCoreID()) + "] BLE : onDisconnect: ");
        Serial.println(_psbrick->_address->toString().c_str());
        _psbrick->_isDiscovered = false;
        _psbrick->_isConnected = false;
        _psbrick->Drive(0, 0, 0, 0);
        if (_psbrick->_driveTaskHandle != NULL)
        {
            vTaskDelete(_psbrick->_driveTaskHandle);
        }

        pclient->disconnect();
    }
}
