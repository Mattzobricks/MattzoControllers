#pragma once

#include <ArduinoJson.h>
#include <SPIFFS.h>

class MCJsonConfig
{
  public:
    // Reads a JSON document of max. 4k.
    static DynamicJsonDocument ReadJsonFile(const char *jsonFilePath);
};