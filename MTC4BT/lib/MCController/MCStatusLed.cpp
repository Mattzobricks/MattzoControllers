#include <Arduino.h>

#include "MCStatusLed.h"
#include "MCController.h"

MCStatusLed::MCStatusLed(int led_pin, bool inverted)
    : MCLedBase(led_pin, inverted)
{
    _on = false;
}

// Updates the status of the LED based on the controller connection status (WiFi and MQTT).
void MCStatusLed::Update()
{
    unsigned long t = millis();

    switch (MCController::GetConnectionStatus())
    {
    case uninitialized:
    case initializing:
        // Two flashes per second.
        Write(((t % 500) < 50) ^ _on);
        break;
    case connecting_wifi:
        // One short flash per second (on 10%).
        Write(((t % 1000) < 100) ^ _on);
        break;
    case connecting_mqtt:
        // Blink (on 50%).
        Write(((t % 1000) < 500) ^ _on);
        break;
    case connected:
        // Off.
        Write(false);
        break;
    }
}