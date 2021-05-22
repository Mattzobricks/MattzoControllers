#include "MCController.h"
#include "DeviceConfiguration.h"
#include "MCStatusLed.h"
#include "Fn.h"
#include "MCLed.h"
#include "log4MC.h"

MCController::MCController()
{
}

MCConnectionStatus MCController::GetConnectionStatus()
{
    if (MattzoWifiClient::GetStatus() == WL_UNINITIALIZED)
    {
        return MCConnectionStatus::uninitialized;
    }

    if (MattzoWifiClient::GetStatus() == WL_INITIALIZING)
    {
        return MCConnectionStatus::initializing;
    }

    if (MattzoWifiClient::GetStatus() != WL_CONNECTED)
    {
        return MCConnectionStatus::connecting_wifi;
    }

    if (MattzoMQTTSubscriber::GetStatus() != MQTT_CONNECTED)
    {
        return MCConnectionStatus::connecting_mqtt;
    }

    return MCConnectionStatus::connected;
}

void MCController::BaseSetup(MCConfiguration *config)
{
    // Setup controller configuration.
    _config = config;
    _ebrake = false;

    // Initialize any status LEDs.
    initStatusLeds();
}

void MCController::BaseLoop()
{
    // If we're not already emergency braking, then emergency brake when the controller is not connected.
    if (!_ebrake && GetConnectionStatus() != MCConnectionStatus::connected)
    {
        SetEmergencyBrake(true);
    }

    // Update leds.
    for (MCLedBase *led : Leds)
    {
        led->Update();
    }
}

MCLedBase *MCController::GetLed(int pin, bool inverted)
{
    for (MCLedBase *led : Leds)
    {
        if (led->GetPin() == pin)
        {
            return led;
        }
    }

    // If not found, define, initialize and add a new LED.
    MCLed *led = new MCLed(pin, inverted);
    Leds.push_back(led);
    return led;
}

bool MCController::GetEmergencyBrake()
{
    return _ebrake;
}

void MCController::SetEmergencyBrake(const bool enabled)
{
    _ebrake = enabled;
    HandleEmergencyBrake(_ebrake);
}

void MCController::initStatusLeds()
{
    // Find the ESP pin configured for the "status" function.
    for (Fn *fn : getFunctions(MCFunction::Status))
    {
        DeviceConfiguration *ledConfig = fn->GetDeviceConfiguration();
        log4MC::vlogf(LOG_INFO, "Ctrl: Found status led attached to ESP pin %u. Initializing...", ledConfig->GetAddressAsEspPinNumber());
        MCStatusLed *statusLed = new MCStatusLed(ledConfig->GetAddressAsEspPinNumber(), ledConfig->IsInverted());
        Leds.push_back(statusLed);
    }
}

std::vector<Fn *> MCController::getFunctions(MCFunction f)
{
    std::vector<Fn *> functions;

    Fn *fn;
    for (int i = 0; i < _config->Functions.size(); i++)
    {
        fn = _config->Functions.at(i);
        if (fn->GetFunction() == f)
        {
            functions.push_back(fn);
        }
    }

    return functions;
}