#include "MCController.h"
#include "DeviceConfiguration.h"
#include "MCStatusLed.h"
#include "Fn.h"
#include "MCLed.h"
#include "log4MC.h"

MCController::MCController(MCConfiguration *config)
{
    _config = config;

    _ebrake = false;
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

void MCController::Setup()
{
    // Initialize any status LEDs.
    initStatusLeds();

    // Start inner task loop (will controle status and LEDs).
    xTaskCreatePinnedToCore(this->loop, "ControllerLoop", 2048, this, 1, NULL, 1);
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

bool MCController::GetEmergencyBreak()
{
    return _ebrake;
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

void MCController::loop(void *parm)
{
    MCController *controller = (MCController *)parm;

    for (;;)
    {
        bool isDisconnected = controller->GetConnectionStatus() != MCConnectionStatus::connected;
        if (isDisconnected != controller->_ebrake)
        {
            controller->_ebrake = isDisconnected;
            controller->EmergencyBreak(controller->_ebrake);
        }

        for (MCLedBase *led : controller->Leds)
        {
            led->Update();
        }
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