#include "MCLoggingConfiguration.h"
#include "MCMQTTConfiguration.h"
#include "MCWiFiConfiguration.h"

struct MCNetworkConfiguration {
  public:
	MCLoggingConfiguration *Logging;
	MCWiFiConfiguration *WiFi;
	MCMQTTConfiguration *MQTT;
};