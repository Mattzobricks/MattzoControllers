#include <Arduino.h>

#include "MCStatusLED.h"
#include "MattzoWiFiClient.h"
#include "MattzoMQTTSubscriber.h"

MCStatusLED::MCStatusLED(int led_pin, bool inverted)
    : MCLed(led_pin, inverted), _statusLEDState{false}
{
}

void MCStatusLED::UpdateByStatus()
{
    unsigned long t = millis();

    switch (getConnectionStatus())
    {
    case uninitialized:
    case initializing:
        // two flaseshes per second.
        Switch(((t % 500) < 50) ^ _statusLEDState);
        break;
    case connecting_wifi:
        // one short flash per second (on 10%).
        Switch(((t % 1000) < 100) ^ _statusLEDState);
        break;
    case connecting_mqtt:
        // blink (on 50%).
        Switch(((t % 1000) < 500) ^ _statusLEDState);
        break;
    case connected:
        // off.
        Switch(false);
        break;
    }
}

MCConnectionStatus MCStatusLED::getConnectionStatus()
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