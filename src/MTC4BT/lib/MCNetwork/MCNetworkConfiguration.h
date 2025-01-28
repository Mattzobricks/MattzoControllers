#include "MCLoggingConfiguration.h"
#include "MCMQTTConfiguration.h"
#include "MCWiFiConfiguration.h"

/*
  networkTypes: 
   - "wired", the "old" W5500 wired
   - "waveshare-ESP32-S3-ETH"
   - "wireless"

*/


struct MCNetworkConfiguration {
  public:
	std::string otaPassword;
	std::string networkType;
  std::string hostname;
	MCLoggingConfiguration *Logging;
	MCWiFiConfiguration *WiFi;
	MCMQTTConfiguration *MQTT;
};