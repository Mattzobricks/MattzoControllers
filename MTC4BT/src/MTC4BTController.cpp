#include "MTC4BTController.h"

MTC4BTController::MTC4BTController(MTC4BTConfiguration *config)
{
    _config = config;
}

DeviceConfiguration *MTC4BTController::GetStatusLed()
{
    // Find the ESP pin configured for the "status" function, making sure there's a light (led) attached.
    for (int i = 0; i < _config->Functions.size(); i++)
    {
        Fn *fn = _config->Functions.at(i);
        if (fn->GetFunction() == MTC4BTFunction::Status)
        {
            // Found device of requested type.
            return fn->GetDevice();
        }
    }

    return nullptr;
}