#include "MTC4BTController.h"
#include "MCLed.h"
#include "enums.h"
#include "log4MC.h"

MTC4BTController::MTC4BTController(MTC4BTConfiguration *config)
{
    _config = config;
}

DeviceConfiguration *MTC4BTController::GetStatusLed()
{
    // Find the ESP pin configured for the "status" function.
    for (Fn *fn : getFunctions(MTC4BTFunction::Status))
    {
        // Return first device of type status led.
        return fn->GetDeviceConfiguration();
    }

    return nullptr;
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
        log4MC::vlogf(LOG_DEBUG, "Loco with address '%u' is not under our control. Fn command ignored.", locoAddress);
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
            log4MC::vlogf(LOG_DEBUG, "Controller handling function %u for pin %u.", ledConfig->GetAttachedDeviceType(), ledConfig->GetAddressAsEspPinNumber());
            MCLed *led = getLed(ledConfig->GetAddressAsEspPinNumber(), ledConfig->IsInverted());
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

MCLed *MTC4BTController::getLed(int pin, bool inverted)
{
    for (MCLed *led : Leds)
    {
        if(led->GetPin() == pin){
            return led;
        }
    }

    MCLed * led = new MCLed(pin, inverted);
    Leds.push_back(led);
    return led;
}