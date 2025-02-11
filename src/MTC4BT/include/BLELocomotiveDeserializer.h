#pragma once

#include "MTC4BTConfiguration.h"
#include "log4MC.h"
#include <ArduinoJson.h>

class BLELocomotiveDeserializer
{
  public:
	// Reads a loco configuration JSON document of max. 4k.
	static BLELocomotiveConfiguration *Deserialize(JsonObject locoConfig, std::vector<MCChannelConfig *> espPins, int16_t defaultPwrIncStep, int16_t defaultPwrDecStep);
};