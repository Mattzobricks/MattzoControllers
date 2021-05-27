#include "MController.h"
#include "BLELocomotive.h"
#include "SBrickHub.h"
#include "PUHub.h"
#include "MCLightController.h"
#include "log4MC.h"

BLELocomotive::BLELocomotive(BLELocomotiveConfiguration *config, MController *controller)
{
    _config = config;
    _controller = controller;

    initLights();
    initHubs();
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

    for (int i = 0; i < Hubs.size(); i++)
    {
        BLEHub *hub = Hubs.at(i);
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

    for (int i = 0; i < Hubs.size(); i++)
    {
        BLEHub *hub = Hubs.at(i);
        hub->Drive(minSpeed, speed);
    }
}

std::vector<Fn *> BLELocomotive::GetFn(MCFunction func)
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
    BLEHub *hub = getHubByAddress(fn->GetPortConfiguration()->GetParentAddress());
    if (hub)
    {
        hub->HandleFn(fn, on);
    }
}

void BLELocomotive::BlinkLights(int durationInMs)
{
    if (!AllHubsConnected())
    {
        // Ignore blink request.
        //log4MC::vlogf(LOG_INFO, "Loco: %s ignored blink lights request because not all its hubs are connected (yet).", _config->_name.c_str());
        return;
    }

    // Handle blink on lights attached to channels of our hubs.
    for (int i = 0; i < Hubs.size(); i++)
    {
        Hubs.at(i)->BlinkLights(durationInMs);
    }
}

void BLELocomotive::EmergencyBrake(const bool enabled)
{
    if (!AllHubsConnected())
    {
        // Ignore function command.
        //log4MC::vlogf(LOG_INFO, "Loco: %s ignored e-brake command because not all its hubs are connected (yet).", _config->_name.c_str());
        return;
    }

    // if (enabled)
    // {
    //     log4MC::vlogf(LOG_WARNING, "Loco: %s e-braking on all hubs.", _config->_name.c_str());
    // }
    // else
    // {
    //     log4MC::vlogf(LOG_WARNING, "Loco: %s releasing e-brake on all hubs.", _config->_name.c_str());
    // }

    // Handle e-brake on our leds connected to ESP pins of the controller.
    for (MCLedBase *led : _espLeds)
    {
        if (enabled)
        {
            // Blink when e-braking.
            led->Write(MCLightController::Blink());
        }
        else
        {
            // Switch back to normal mode.
            led->Switch(led->IsOn());
        }
    }

    // Handle e-brake on lights attached to channels of our hubs.
    for (int i = 0; i < Hubs.size(); i++)
    {
        Hubs.at(i)->EmergencyBrake(enabled);
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
    return Hubs.size();
}

BLEHub *BLELocomotive::GetHub(uint index)
{
    return Hubs.at(index);
}

bool BLELocomotive::GetAutoLightsEnabled()
{
    return _config->_autoLightsEnabled;
}

void BLELocomotive::initLights()
{
    for (Fn *fn : _config->_functions)
    {
        PortConfiguration *deviceConfig = fn->GetPortConfiguration();
        if (deviceConfig->GetAttachedDeviceType() == DeviceType::Light)
        {
            // Ask controller to create an led for us and keep a reference to it.
            _espLeds.push_back(_controller->GetLed(deviceConfig->GetAddressAsEspPinNumber(), deviceConfig->IsInverted()));
        }
    }
}

void BLELocomotive::initHubs()
{
    for (BLEHubConfiguration *hubConfig : _config->_hubs)
    {
        switch (hubConfig->HubType)
        {
        case BLEHubType::SBrick:
            Hubs.push_back(new SBrickHub(hubConfig, _config->_speedStep, _config->_brakeStep));
            break;
        case BLEHubType::PU:
            Hubs.push_back(new PUHub(hubConfig, _config->_speedStep, _config->_brakeStep));
            break;
        }
    }

    // log4MC::vlogf(LOG_INFO, "Loco: %s hub config initialized.", _config->_name.c_str());
}

BLEHub *BLELocomotive::getHubByAddress(std::string address)
{
    for (BLEHub *hub : Hubs)
    {
        if (hub->GetAddress().compare(address) == 0)
        {
            return hub;
        }
    }

    return nullptr;
}

std::vector<Fn *> BLELocomotive::getFunctions(MCFunction f)
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