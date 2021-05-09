#include <Arduino.h>
#include <MCStatusLED.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

MCStatusLED::MCStatusLED(int led_pin, bool inverted)
    : status_led_pin{led_pin}, statusLEDState{false}, inverted{inverted}
{
    pinMode(led_pin,OUTPUT);
}

MCConnectionStatus MCStatusLED::getConnectionStatus()
{
    if (!mcconfig.haveConfig) {
        return noconfig;
    }
    if (WiFi.status() != WL_CONNECTED) {
        return connecting_wifi;
    }
    //if (!mqttClient.connected()) {
        return connecting_mqtt;
    //}
    return connected;
}

void MCStatusLED::setStatusLED(bool ledState)
{
    if (status_led_pin > 0) {
        if (!inverted) {
            digitalWrite(status_led_pin, ledState ? HIGH : LOW);
        } else {
            digitalWrite(status_led_pin, ledState ? LOW : HIGH);
        }
    }
}

void MCStatusLED::updateStatusLED()
{
    unsigned long t = millis();

    switch (getConnectionStatus()) {
    case noconfig:
        // two flaseshes per second
        setStatusLED(((t % 500) < 50) ^ statusLEDState);
        break;
    case connecting_wifi:
        // flash
        setStatusLED(((t % 1000) < 100) ^ statusLEDState);
        break;
    case connecting_mqtt:
        // blink
        setStatusLED(((t % 1000) < 500) ^ statusLEDState);
        break;
    case connected:
        // off
        setStatusLED(statusLEDState);
        break;
    }
}