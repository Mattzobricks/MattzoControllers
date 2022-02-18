#pragma once

#ifdef ESP32
#include <SPIFFS.h>
#define CONFIGFS SPIFFS
#else
#include <LittleFS.h>
#define CONFIGFS LittleFS
#endif
// CONFIGFS is needed in otastart, to make sure SPIFFS or LittleFS is unmounted!