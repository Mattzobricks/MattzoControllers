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
    if (!IsEnabled())
    {
        return false;
    }

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

std::vector<Fn *> BLELocomotive::GetFn(MTC4BTFunction func)
{
    return getFunctions(func);
}

void BLELocomotive::HandleFn(Fn *fn, const bool on)
{
    if (!AllHubsConnected())
    {
        // Ignore function command.
        log4MC::vlogf(LOG_INFO, "Loco: %s ignored function command because not all its hubs are connected (yet).", _config->_name.c_str());

        // Return success anyway, because we don't want the controller to handle the function.
        return;
    }

    // Ask hub to handle function.
    BLEHub *hub = getHubByAddress(fn->GetDeviceConfiguration()->GetParentAddress());
    if (hub)
    {
        hub->HandleFn(fn, on);
    }
}

void BLELocomotive::EmergencyBreak(const bool enabled)
{
    if (!AllHubsConnected())
    {
        // Ignore function command.
        log4MC::vlogf(LOG_INFO, "Loco: %s ignored e-brake command because not all its hubs are connected (yet).", _config->_name.c_str());
        return;
    }

    // Set e-break on all hubs.
    if (enabled)
    {
        log4MC::vlogf(LOG_WARNING, "Loco: %s e-braking on all hubs.", _config->_name.c_str());
    }
    else
    {
        log4MC::vlogf(LOG_WARNING, "Loco: %s releasing e-brake on all hubs.", _config->_name.c_str());
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

BLEHub *BLELocomotive::getHubByAddress(std::string address)
{
    for (BLEHub *hub : _hubs)
    {
        if (hub->GetAddress().compare(address) == 0)
        {
            return hub;
        }
    }

    return nullptr;
}

std::vector<Fn *> BLELocomotive::getFunctions(MTC4BTFunction f)
{
    std::vector<Fn *> functions;

    if (AllHubsConnected())
    {
        for (Fn *fn : _config->_functions)
        {
            if (fn->GetFunction() == f)
            {
                functions.push_back(fn);
            }
        }
    }

    return functions;
}