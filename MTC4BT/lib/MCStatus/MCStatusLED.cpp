#include <Arduino.h>

#include "MCStatusLED.h"
#include "MattzoWiFiClient.h"
#include "MattzoMQTTSubscriber.h"

MCStatusLED::MCStatusLED(int led_pin, bool inverted)
    : _status_led_pin{led_pin}, _statusLEDState{false}, _inverted{inverted}
{
    Serial.print("Status pin inverted: ");
    Serial.println(inverted);
    pinMode(led_pin, OUTPUT);
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

void MCStatusLED::setStatusLED(bool on)
{
    if (_status_led_pin > 0)
    {
        if (!_inverted)
        {
            digitalWrite(_status_led_pin, on ? HIGH : LOW);
        }
        else
        {
            digitalWrite(_status_led_pin, on ? LOW : HIGH);
        }
    }
}

void MCStatusLED::UpdateStatusLED()
{
    unsigned long t = millis();

    switch (getConnectionStatus())
    {
    case uninitialized:
    case initializing:
        // two flaseshes per second.
        setStatusLED(((t % 500) < 50) ^ _statusLEDState);
        break;
    case connecting_wifi:
        // one short flash per second (on 10%).
        setStatusLED(((t % 1000) < 100) ^ _statusLEDState);
        break;
    case connecting_mqtt:
        // blink (on 50%).
        setStatusLED(((t % 1000) < 500) ^ _statusLEDState);
        break;
    case connected:
        // off.
        setStatusLED(false);
        break;
    }
}