#include "processAddress.h"
#include "MCJsonConfig.h"
#include "log4MC.h"

processAddress::processAddress(const char *configFilePath)
{
    JsonDocument doc = MCJsonConfig::ReadJsonFile(configFilePath);
    if (!doc.isNull())
    {
        // read the configuration
        log4MC::debug("Reading lookuptable");
    } else{
        log4MC::debug("No lookuptable found, or empty");
    }
}

std::string processAddress::process(const std::string inStr)
{
	std::string outStr;
	outStr = inStr; // for now copy the input to the output
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