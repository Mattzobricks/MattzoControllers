#pragma once

#include <Arduino.h>
#include <XmlParser.h>

#include "MTC4BTController.h"

// Class used to translate MQTT messages to BLE commands.
class MattzoBLEMQTTHandler
{

public:
    // Handle the given MQTT message and apply it to the applicable loco.
    static void Handle(const char *message, MTC4BTController *controller);

private:
    static void handleSys(const char *message, MTC4BTController *controller);
    static void handleLc(const char *message, MTC4BTController *controller);
    static void handleFn(const char *message, MTC4BTController *controller);
};