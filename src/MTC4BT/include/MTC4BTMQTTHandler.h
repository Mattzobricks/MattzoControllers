#pragma once

#include <Arduino.h>
#include <XmlParser.h>

#include "rocrailitems/lclist.h"

// Class used to translate MQTT messages to BLE commands.
class MTC4BTMQTTHandler
{
  public:
    // Handles the given MQTT message and applies it to the applicable loco(s).
    static void Handle(const char *message);
    static void infoHandle(const char *message);

    static void pubGetShortLcList();
    static void pubGetLcInfo(char *locid);
    static void pubLcSpeed(char *locid, int addr, long locV);

  private:
    static void handleSys(const char *message);
    static void handleLc(const char *message);
    static void handleFn(const char *message);
    static void handleLCList(const char *message);
    static void handleInfoLc(const char *message);

    static lc *getCurrentLcSpeed(const char *message, bool has_previd);
};