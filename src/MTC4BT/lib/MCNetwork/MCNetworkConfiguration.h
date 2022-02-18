#include "MCLoggingConfiguration.h"
#include "MCWiFiConfiguration.h"
#include "MCMQTTConfiguration.h"

struct MCNetworkConfiguration
{
public:
    MCLoggingConfiguration *Logging;
    MCWiFiConfiguration *WiFi;
    MCMQTTConfiguration *MQTT;
};