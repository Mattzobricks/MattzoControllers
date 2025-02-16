#pragma once

#include "BLERemoteConfiguration.h"
#include "MTC4BTConfiguration.h"
#include "log4MC.h"
#include "processAddress.h"
#include <ArduinoJson.h>

class BLERemoteDeserializer
{
  public:
	// Reads a loco configuration JSON document of max. 4k.
	static BLERemoteConfiguration *Deserialize(JsonObject remoteConfig, int16_t defaultPwrIncStep, int16_t defaultPwrDecStep, processAddress *processor);
};