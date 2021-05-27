#include "MCLightController.h"

bool MCLightController::TwoFlashesPerSecond()
{
    return on(2, 50);
}

bool MCLightController::OneFlashPerSecond()
{
    return on(1, 100);
}

bool MCLightController::Blink()
{
    return on(1, 500);
}

bool MCLightController::on(uint flashesPerSecond, uint flashDurationInMs)
{
    return (millis() % (1000 / flashesPerSecond)) < flashDurationInMs;
}