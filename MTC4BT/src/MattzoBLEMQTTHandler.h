#pragma once

#include <Arduino.h>
#include <XmlParser.h>

#include "BLEHub.h"
#include "BLELocomotive.h"

// Class used to translate MQTT messages to BLE commands.
class MattzoBLEMQTTHandler
{

public:
    // Handle the given MQTT message and apply it to the applicable loco.
    static void Handle(const char *message, std::vector<BLELocomotive *> locos);

private:
    static void handleSys(const char *message, std::vector<BLELocomotive *> locos);
    static void handleLc(const char *message, std::vector<BLELocomotive *> locos);
    static void handleFn(const char *message, std::vector<BLELocomotive *> locos);
};