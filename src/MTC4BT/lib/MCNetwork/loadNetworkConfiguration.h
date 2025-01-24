#pragma once

#include "MCNetworkConfiguration.h"
#include <string.h>

MCNetworkConfiguration *loadNetworkConfiguration(const char *configFilePath)
{
	// New up a configation object, so we can set its properties.
	MCNetworkConfiguration *config = new MCNetworkConfiguration();

	// Initialize file system.
	if (!SPIFFS.begin(true)) {
		Serial.println("An error has occurred while mounting SPIFFS");
		return config;
	}

	// Read JSON network config file.
	JsonDocument doc = MCJsonConfig::ReadJsonFile(configFilePath);
	JsonObject networkConfig = doc["network"];
	config->otaPassword = networkConfig["otaPassword"].as<std::string>();
	config->networkType = networkConfig["type"].as<std::string>();

	if (config->networkType == "") {
		config->networkType = "wireless";
	} else if (config->networkType != "wired" || config->networkType != "wireless" || config->networkType != "waveshare-ESP32-S3-ETH") {
		log4MC::vlogf(LOG_ERR, "Config: UNKOWN NETWORK TYPE: %s, falling back to wireless!",config->networkType.c_str());
		config->networkType = "wireless";
	}

	// Read logging configuration.
	MCLoggingConfiguration *logging = new MCLoggingConfiguration();
	/// Read Serial log configuration
	logging->Serial = new MCLoggingSerialConfiguration();

	JsonObject loggingConfig = doc["logging"];
	logging->MinLevel = loggingConfig["min_level"] | "info"; // keep it around for now
	const char *minLevel = loggingConfig["min_level"] | "info";
	if (minLevel) {
		if (strcmp(minLevel, "debug") == 0) {
			logging->mask = LOG_DEBUG;
		} else if (strcmp(minLevel, "info") == 0) {
			logging->mask = LOG_INFO;
		} else if (strcmp(minLevel, "warning") == 0) {
			logging->mask = LOG_WARNING;
		} else if (strcmp(minLevel, "error") == 0) {
			logging->mask = LOG_ERR;
		} else if (strcmp(minLevel, "fatal") == 0) {
			logging->mask = LOG_CRIT;
		} else {
			logging->mask = LOG_EMERG;
		}
		logging->Serial->min_level = logging->mask;
		logging->mask = ((1 << ((logging->mask) + 1)) - 1);
	}

	logging->Serial->Enabled = loggingConfig["serial"]["enabled"] | true;

	// Read Syslog configuration.
	JsonObject syslogConfig = loggingConfig["syslog"];
	logging->SysLog = new MCLoggingSyslogConfiguration();
	logging->SysLog->Enabled = syslogConfig["enabled"] | false;
	if (logging->SysLog->Enabled) {
		logging->SysLog->ServerAddress = syslogConfig["server"].as<std::string>();
		logging->SysLog->ServerPort = syslogConfig["port"] | 514;
		logging->SysLog->AppName = syslogConfig["appname"].as<std::string>();

		logging->SysLog->mask = logging->SysLog->mask;
	}

	// Attach logging configuration.
	config->Logging = logging;

	// Read WiFi configuration.
	MCWiFiConfiguration *wifi = new MCWiFiConfiguration();
	JsonObject wifiConfig = doc["wifi"];
	wifi->SSID = wifiConfig["SSID"].as<std::string>();
	wifi->password = wifiConfig["password"].as<std::string>();
	wifi->hostname = wifiConfig["hostname"].as<std::string>();
	wifi->otaPassword = wifiConfig["otaPassword"].as<std::string>();
	wifi->DailyBetweenConnectAttempsInMs = wifiConfig["wait"] | 1000;

	// Attach WiFi configuration.
	config->WiFi = wifi;

	// Read MQTT configuration.
	MCMQTTConfiguration *mqtt = new MCMQTTConfiguration();
	JsonObject mqttConfig = doc["mqtt"];
	mqtt->ServerAddress = mqttConfig["broker"].as<std::string>();
	mqtt->ServerPort = mqttConfig["port"] | 1883;
	mqtt->KeepAlive = mqttConfig["keepalive"] | 10;
	mqtt->Ping = mqttConfig["ping"] | 0;
	// Attach MQTT configuration.
	config->MQTT = mqtt;

	// Return MCNetworkConfiguration object.
	return config;
}