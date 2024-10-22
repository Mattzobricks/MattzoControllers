#pragma once

#include <Arduino.h>
#include <XmlParser.h>

// Class used to translate MQTT messages to BLE commands.
class MTC4BTMQTTHandler
{
  public:
    // Handles the given MQTT message and applies it to the applicable loco(s).
    static void Handle(const char *message);

  private:
    static void handleSys(const char *message);
    static void handleLc(const char *message);
    static void handleFn(const char *message);
};