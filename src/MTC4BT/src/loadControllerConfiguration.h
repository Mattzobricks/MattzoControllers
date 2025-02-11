#pragma once

#include "BLELocomotiveDeserializer.h"
#include "BLERemoteDeserializer.h"

#define DEFAULT_CONTROLLER_NAME "MTC4BT"
#define DEFAULT_PWR_INC_STEP 10
#define DEFAULT_PWR_DEC_STEP 10

MTC4BTConfiguration *loadControllerConfiguration(const char *configFilePath)
{
	// New up a configation object, so we can set its properties.
	MTC4BTConfiguration *config = new MTC4BTConfiguration();
	BLERemoteConfiguration *deserializedRemoteConfig;
	// Initialize file system.
	if (!SPIFFS.begin(true)) {
		Serial.println("Config: An error has occurred while mounting SPIFFS");
		return config;
	}

	// Read JSON controller config file.
	JsonDocument doc = MCJsonConfig::ReadJsonFile(configFilePath);

	// Read controller name.
	const char *controllerName = doc["name"] | DEFAULT_CONTROLLER_NAME;
	config->ControllerName = (char *)malloc(strlen(controllerName) + 2);
	strncpy(config->ControllerName, controllerName, strlen(controllerName) + 1);

	log4MC::vlogf(LOG_INFO, "Config: Read controller name: %s", config->ControllerName);

	int16_t pwrIncStep = doc["pwrIncStep"] | DEFAULT_PWR_INC_STEP;
	int16_t pwrDecStep = doc["pwrDecStep"] | DEFAULT_PWR_DEC_STEP;

	// Iterate over ESP pins and copy values from the JsonDocument to MCChannelConfig objects.
	JsonArray espPinConfigs = doc["espPins"].as<JsonArray>();
	for (JsonObject espPinConfig : espPinConfigs) {
		// Use pin number as its device address.
		const std::string address = espPinConfig["pin"];
		int16_t pinPwrIncStep = espPinConfig["pwrIncStep"] | pwrIncStep;
		int16_t pinPwrDecStep = espPinConfig["pwrDecStep"] | pwrDecStep;
		int16_t pinPwr = espPinConfig["power"] | 100;
		const bool isInverted = espPinConfig["inverted"] | false;
		const std::string attachedDevice = espPinConfig["attachedDevice"] | "nothing";

		MCChannel *espChannel = new MCChannel(ChannelType::EspPinChannel, address);
		config->EspPins.push_back(new MCChannelConfig(espChannel, false, pinPwrIncStep, pinPwrDecStep, isInverted, pinPwr, deviceTypeMap()[attachedDevice]));
	}
	log4MC::vlogf(LOG_INFO, "Config: Read ESP pin configuration (%u).", config->EspPins.size());

	// Iterate over loco configs and copy values from the JsonDocument to BLELocomotiveConfiguration objects.
	JsonArray locoConfigs = doc["locos"].as<JsonArray>();
	for (JsonObject locoConfig : locoConfigs) {
		// Read if loco is enabled.
		const bool enabled = locoConfig["enabled"] | true;
		if (!enabled) {
			// Skip if loco is not enabled.
			continue;
		}

		config->LocoConfigs.push_back(BLELocomotiveDeserializer::Deserialize(locoConfig, config->EspPins, pwrIncStep, pwrDecStep));
	}

	// Iterate over remote configs and copy values from the JsonDocument to BLERemoteConfiguration objects.
	JsonArray remoteConfigs = doc["remotes"].as<JsonArray>();
	for (JsonObject remoteConfig : remoteConfigs) {
		// Read if loco is enabled.
		const bool enabled = remoteConfig["enabled"] | true;
		if (!enabled) {
			// Skip if loco is not enabled.
			continue;
		}
		deserializedRemoteConfig = BLERemoteDeserializer::Deserialize(remoteConfig, pwrIncStep, pwrDecStep);
		if (deserializedRemoteConfig)
			config->RemoteConfigs.push_back(deserializedRemoteConfig);
	}

	// Read loco config files.
	JsonArray locoConfigFiles = doc["locoConfigs"].as<JsonArray>();
	for (int i = 0; i < locoConfigFiles.size(); i++) {
		const std::string locoConfigFile = locoConfigFiles[i];

		// Read JSON controller config file.
		JsonDocument locoConfigDoc = MCJsonConfig::ReadJsonFile(locoConfigFile.c_str());
		JsonObject locoConfig = locoConfigDoc.as<JsonObject>();

		// Read if loco is enabled.
		const bool enabled = locoConfig["enabled"] | true;
		if (!enabled) {
			// Skip if loco is not enabled.
			continue;
		}

		config->LocoConfigs.push_back(BLELocomotiveDeserializer::Deserialize(locoConfig, config->EspPins, pwrIncStep, pwrDecStep));
	}

	// Read remote config files.
	JsonArray remoteConfigFiles = doc["remoteConfigs"].as<JsonArray>();
	for (int i = 0; i < remoteConfigFiles.size(); i++) {
		const std::string remoteConfigFile = remoteConfigFiles[i];

		// Read JSON controller config file.
		JsonDocument remoteConfigDoc = MCJsonConfig::ReadJsonFile(remoteConfigFile.c_str());
		JsonObject remoteConfig = remoteConfigDoc.as<JsonObject>();

		// Read if loco is enabled.
		const bool enabled = remoteConfig["enabled"] | true;
		if (!enabled) {
			// Skip if loco is not enabled.
			continue;
		}
		deserializedRemoteConfig = BLERemoteDeserializer::Deserialize(remoteConfig, pwrIncStep, pwrDecStep);
		if (deserializedRemoteConfig)
			config->RemoteConfigs.push_back(deserializedRemoteConfig);
	}

	// Return MTC4BTConfiguration object.
	return config;
}