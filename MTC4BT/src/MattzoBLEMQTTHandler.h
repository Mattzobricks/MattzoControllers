#pragma once

#include <Arduino.h>

#include "BLEHub.h"

// Class used to translate MQTT messages to BLE commands.
class MattzoBLEMQTTHandler
{

public:
    static const int16_t LIGHT_ON_SPEED = 100;
    static const int16_t LIGHT_OFF_SPEED = 0;

    // Methods

    // Handle the given MQTT message and apply it to the applicable SBrick(s).
    static void Handle(const char *message, ulong hubCount, BLEHub *hubs[]);

private:
    // Methods

    static bool isNodeType(const String message, const char *nodeName);
    static void handleSys(const String message, ulong hubCount, BLEHub *hubs[]);
    static void handleLc(const String message, ulong hubCount, BLEHub *hubs[]);
    static void handleFn(const String message, ulong hubCount, BLEHub *hubs[]);
    static String getAttr(const String message, const String attrName);
};