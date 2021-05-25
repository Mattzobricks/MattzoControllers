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

void MCController::Setup(MCConfiguration *config)
{
    // Setup controller configuration.
    _config = config;
    _ebrake = false;

    // Initialize any status LEDs.
    initStatusLeds();
}

void MCController::Loop()
{
    // E-brake is enabled when specifically requested (through MQTT) or when the controller is not connected.
    bool ebrakeEnabled = _ebrake || GetConnectionStatus() != MCConnectionStatus::connected;

    // Update leds (taking e-brake into account).
    for (MCLedBase *led : Leds)
    {
        led->Update(ebrakeEnabled);
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
}

void MCController::HandleFn(Fn *fn, const bool on)
{
    DeviceConfiguration *ledConfig = fn->GetDeviceConfiguration();
    log4MC::vlogf(LOG_DEBUG, "Ctrl: Handling function %u for pin %u.", ledConfig->GetAttachedDeviceType(), ledConfig->GetAddressAsEspPinNumber());
    MCLedBase *led = GetLed(ledConfig->GetAddressAsEspPinNumber(), ledConfig->IsInverted());

    if (led)
    {
        led->Switch(on);
    }
}

void MCController::initStatusLeds()
{
    // Find the ESP pins configured with the "status" led function.
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