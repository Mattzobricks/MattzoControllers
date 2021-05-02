#pragma once

#include <Arduino.h>
#include <XmlParser.h>

#include "BLEHub.h"
#include "BLELocomotive.h"

// Class used to translate MQTT messages to BLE commands.
class MattzoBLEMQTTHandler
{

public:
    static const int16_t LIGHT_ON_SPEED = 100;
    static const int16_t LIGHT_OFF_SPEED = 0;

    // Methods

    // Handle the given MQTT message and apply it to the applicable loco.
    static void Handle(const char *message, ulong locoCount, BLELocomotive *locos[]);

private:
    // Methods

    static void handleSys(const char *message, ulong locoCount, BLELocomotive *locos[]);
    static void handleLc(const char *message, ulong locoCount, BLELocomotive *locos[]);
    static void handleFn(const char *message, ulong locoCount, BLELocomotive *locos[]);
};