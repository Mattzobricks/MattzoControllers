#include "BLELocomotive.h"
#include "SBrickHub.h"
#include "PUHub.h"
#include "log4MC.h"

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
        log4MC::vlogf(LOG_INFO, "Loco: %s ignored drive command because not all its hubs are connected (yet).", _config->_name.c_str());
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
        // Ignore function command.
        log4MC::vlogf(LOG_INFO, "Loco: %s ignored function command because not all its hubs are connected (yet).", _config->_name.c_str());
        return;
    }

    // Convert function number to string "fx";
    char fnName[3];
    sprintf(fnName, "f%u", fn);
    MTC4BTFunction func = functionMap()[fnName];

    for (int i = 0; i < _config->_functions.size(); i++)
    {
        Fn *function = _config->_functions.at(i);
        if (function->GetFunction() == func)
        {
            // Found function definition, determine hub address.
            std::string hubAddress = function->GetDevice()->GetParentAddress();

            // Ask hub to handle function.
            GetHub(hubAddress)->HandleFn(function, on);
        }
    }
}

void BLELocomotive::EmergencyBreak(const bool enabled)
{
    // Set e-break on all hubs.
    if (enabled)
    {
        log4MC::vlogf(LOG_WARNING, "Loco: %s e-breaking on all hubs.", _config->_name.c_str());
    }
    else
    {
        log4MC::vlogf(LOG_WARNING, "Loco: %s releasing e-break on all hubs.", _config->_name.c_str());
    }

    for (int i = 0; i < _hubs.size(); i++)
    {
        _hubs.at(i)->EmergencyBreak(enabled);
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

BLEHub *BLELocomotive::GetHub(std::string address)
{
    for (int i = 0; i < _hubs.size(); i++)
    {
        if (_hubs.at(i)->GetAddress().compare(address) == 0)
        {
            return _hubs.at(i);
        }
    }

    return nullptr;
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
            _hubs.push_back(new SBrickHub(hubConfig, _config->_speedStep, _config->_brakeStep));
            break;
        case BLEHubType::PU:
            _hubs.push_back(new PUHub(hubConfig, _config->_speedStep, _config->_brakeStep));
            break;
        }
    }

    // log4MC::vlogf(LOG_INFO, "Loco: %s hub config initialized.", _config->_name.c_str());
}