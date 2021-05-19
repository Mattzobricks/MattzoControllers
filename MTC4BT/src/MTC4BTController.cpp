#include "MTC4BTController.h"
#include "MCStatusLed.h"
#include "MCLed.h"
#include "enums.h"
#include "log4MC.h"

MTC4BTController::MTC4BTController(MTC4BTConfiguration *config)
{
    _config = config;
    initStatusLeds();
}

void MTC4BTController::EmergencyBreak(const bool enabled)
{
    // Handle e-brake on all locomotives.
    for (int i = 0; i < _config->Locomotives.size(); i++)
    {
        _config->Locomotives.at(i)->EmergencyBreak(enabled);
    }
}

BLELocomotive *MTC4BTController::GetLocomotive(uint address)
{
    BLELocomotive *loco;

    for (int i = 0; i < _config->Locomotives.size(); i++)
    {
        loco = _config->Locomotives.at(i);
        if (loco->GetLocoAddress() == address)
        {
            return loco;
        }
    }

    return nullptr;
}

void MTC4BTController::HandleFn(int locoAddress, MTC4BTFunction f, const bool on)
{
    BLELocomotive *loco = GetLocomotive(locoAddress);
    if (!loco)
    {
        // Not a loco under our control. Ignore message.
        log4MC::vlogf(LOG_DEBUG, "Ctrl: Loco with address '%u' is not under our control. Fn command ignored.", locoAddress);
        return;
    }

    // Get applicable functions from loco.
    for (Fn *fn : loco->GetFn(f))
    {
        // Determine type of hardware.
        switch (fn->GetDeviceConfiguration()->GetHardwareType())
        {
        case HardwareType::EspPin:
        {
            // Handle function locally on the controller.
            DeviceConfiguration *ledConfig = fn->GetDeviceConfiguration();
            log4MC::vlogf(LOG_DEBUG, "Ctrl: Handling function %u for pin %u.", ledConfig->GetAttachedDeviceType(), ledConfig->GetAddressAsEspPinNumber());
            MCLedBase *led = getLed(ledConfig->GetAddressAsEspPinNumber(), ledConfig->IsInverted());
            led->Switch(on);
            break;
        }
        case HardwareType::BleHub:
        {
            // Let loco handle the function.
            loco->HandleFn(fn, on);
            break;
        }
        }
    }
}

void MTC4BTController::initStatusLeds()
{
    // Find the ESP pin configured for the "status" function.
    for (Fn *fn : getFunctions(MTC4BTFunction::Status))
    {
        DeviceConfiguration *ledConfig = fn->GetDeviceConfiguration();
        log4MC::vlogf(LOG_INFO, "Ctrl: Found status led attached to ESP pin %u. Initializing...", ledConfig->GetAddressAsEspPinNumber());
        MCStatusLed *statusLed = new MCStatusLed(ledConfig->GetAddressAsEspPinNumber(), ledConfig->IsInverted());
        Leds.push_back(statusLed);
    }
}

std::vector<Fn *> MTC4BTController::getFunctions(MTC4BTFunction f)
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

MCLedBase *MTC4BTController::getLed(int pin, bool inverted)
{
    for (MCLedBase *led : Leds)
    {
        if (led->GetPin() == pin)
        {
            return led;
        }
    }

    MCLed *led = new MCLed(pin, inverted);
    Leds.push_back(led);
    return led;
}