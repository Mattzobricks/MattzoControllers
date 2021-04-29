#pragma once

#include <Arduino.h>
#include <XmlParser.h>

#include "BLEHub.h"

// Class used to translate MQTT messages to BLE commands.
class MattzoBLEMQTTHandler
{

public:
    static const int16_t LIGHT_ON_SPEED = 100;
    static const int16_t LIGHT_OFF_SPEED = 0;

    // Methods

    // Handle the given MQTT message and apply it to the applicable BLE hub(s).
    static void Handle(const char *message, ulong hubCount, BLEHub *hubs[]);

private:
    // Methods

    static void handleSys(const char *message, ulong hubCount, BLEHub *hubs[]);
    static void handleLc(const char *message, ulong hubCount, BLEHub *hubs[]);
    static void handleFn(const char *message, ulong hubCount, BLEHub *hubs[]);
};