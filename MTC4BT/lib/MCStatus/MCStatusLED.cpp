#include <Arduino.h>

#include "MCStatusLED.h"
// #include "PubSubClient.h"
#include "MattzoWiFiClient.h"
#include "MattzoMQTTSubscriber.h"

MCStatusLED::MCStatusLED(int led_pin, bool inverted)
    : status_led_pin{led_pin}, statusLEDState{false}, inverted{inverted}
{
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
    if (status_led_pin > 0)
    {
        if (!inverted)
        {
            digitalWrite(status_led_pin, on ? HIGH : LOW);
        }
        else
        {
            digitalWrite(status_led_pin, on ? LOW : HIGH);
        }
    }
}

void MCStatusLED::UpdateStatusLED()
{
    switch (getConnectionStatus())
    {
    case uninitialized:
    case initializing:
        // two short flashes per second
        OnTime = 100;
        OffTime = 400;
        break;
    case connecting_wifi:
        // one short flash per second (10%)
        OnTime = 100;
        OffTime = 900;
        break;
    case connecting_mqtt:
        // blink (50%)
        OnTime = 500;
        OffTime = 500;
        break;
    case connected:
        // off
        OnTime = 0;
        OffTime = 0;
        break;
    }

    unsigned long t = millis();

    if ((ledOn == true) && (t - previousMillis >= OnTime))
    {
        ledOn = false;
        previousMillis = t;
        setStatusLED(ledOn);
    }
    else if ((ledOn == false) && (t - previousMillis >= OffTime))
    {
        ledOn = true;
        previousMillis = t;
        setStatusLED(ledOn);
    }
}