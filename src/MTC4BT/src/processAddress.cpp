#include "processAddress.h"
#include "MCJsonConfig.h"
#include "log4MC.h"

processAddress::processAddress(const char *configFilePath)
{
	JsonDocument doc = MCJsonConfig::ReadJsonFile(configFilePath);
	if (!doc.isNull()) {
		// read the configuration
		log4MC::debug("Reading lookuptable");
		JsonArray lookupTable = doc["lookuptable"].as<JsonArray>();
		for (JsonObject lookupItem : lookupTable) {
			const std::string name = lookupItem["name"];
			const std::string address = lookupItem["address"];

			if (lookupItem["name"].is<std::string>() &&
				lookupItem["address"].is<std::string>()) {
				// both items are there, add the item to the lookuptable vector
				lut[name] = address;
			}
		}

	} else {
		log4MC::debug("No lookuptable found, or empty");
	}
}

std::string processAddress::process(const std::string inStr)
{
	std::string outStr;
    auto it = lut.find(inStr);
	if (it != lut.end()) {
        outStr = it->second;
	} else {
		outStr = inStr; // for now copy the input to the output
	}
	// as a last action, convert the address to lowercase
	for (char &c : outStr) {
		if (c >= 'A' && c <= 'Z') {
			// Convert uppercase to lowercase
			// by adding 32
			c += 32;
		}
	}
	return outStr;
}