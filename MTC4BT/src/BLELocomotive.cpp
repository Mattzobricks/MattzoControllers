#include "BLELocomotive.h"
#include "SBrickHub.h"
#include "PUHub.h"

BLELocomotive::BLELocomotive(BLELocomotiveConfiguration *config)
{
    _config = config;

    initHubs(config->_hubs);
}

bool BLELocomotive::IsEnabled()
{
    return _config->_enabled;
}

bool BLELocomotive::AllHubsConnected()
{
    for (int i = 0; i < _hubs.size(); i++)
    {
        BLEHub *hub = _hubs.at(i);
        if (!hub->IsConnected())
        {
            return false;
        }
    }

    return true;
}

void BLELocomotive::Drive(const int16_t minSpeed, const int16_t speed)
{
    if (!AllHubsConnected())
    {
        // Ignore drive command.
        return;
    }

    for (int i = 0; i < _hubs.size(); i++)
    {
        BLEHub *hub = _hubs.at(i);
        hub->Drive(minSpeed, speed);
    }
}

void BLELocomotive::SetFunction(const uint8_t fn, const bool on)
{
    if (!AllHubsConnected())
    {
        // Ignore drive command.
        return;
    }

    // TODO: Implement!
}

void BLELocomotive::EmergencyBreak(const bool enabled)
{
    for (int i = 0; i < _hubs.size(); i++)
    {
        BLEHub *hub = _hubs.at(i);
        hub->EmergencyBreak(enabled);
    }
}

std::string BLELocomotive::GetLocoName()
{
    return _config->_name;
}

uint BLELocomotive::GetLocoAddress()
{
    return _config->_address;
}

uint BLELocomotive::GetHubCount()
{
    return _hubs.size();
}

BLEHub *BLELocomotive::GetHub(uint index)
{
    return _hubs.at(index);
}

bool BLELocomotive::GetAutoLightsEnabled()
{
    return _config->_autoLightsEnabled;
}

void BLELocomotive::initHubs(std::vector<BLEHubConfiguration *> hubConfigs)
{
    for (int i = 0; i < hubConfigs.size(); i++)
    {
        BLEHubConfiguration *hubConfig = hubConfigs.at(i);
        switch (hubConfig->HubType)
        {
        case BLEHubType::SBrick:
            _hubs.push_back(new SBrickHub(hubConfig));
            break;
        case BLEHubType::PU:
            _hubs.push_back(new PUHub(hubConfig));
            break;
        }
    }
}