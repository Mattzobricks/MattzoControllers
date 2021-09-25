#include <Arduino.h>

#include "MCStatusLed.h"
#include "MController.h"
#include "MCLightController.h"

MCStatusLed::MCStatusLed(int led_pin, bool inverted)
    : MCLedBase(led_pin, inverted) {}

// Updates the status of the LED based on the controller connection status (WiFi and MQTT).
void MCStatusLed::Update(bool ebrakeEnabled)
{
    // Status LED ignores the e-brake status and purely looks at the controller connection status.

    switch (MController::GetConnectionStatus())
    {
    case uninitialized:
    case initializing:
        // Two flashes per second.
        Write(MCLightController::TwoFlashesPerSecond());
        break;
    case connecting_wifi:
        // One short flash per second (on 10%).
        Write(MCLightController::OneFlashPerSecond());
        break;
    case connecting_mqtt:
        // Blink (on 50%).
        Write(MCLightController::Blink());
        break;
    case connected:
        // Off.
        Write(false);
        break;
    }
}