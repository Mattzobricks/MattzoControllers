#pragma once

#include "BLELocomotiveDeserializer.h"

#define DEFAULT_CONTROLLER_NAME "MTC4BT"
#define DEFAULT_PWR_INC_STEP 10
#define DEFAULT_PWR_DEC_STEP 10

MTC4BTConfiguration *loadControllerConfiguration(const char *configFilePath)
{
    // New up a configation object, so we can set its properties.
    MTC4BTConfiguration *config = new MTC4BTConfiguration();

    // Initialize file system.
    if (!SPIFFS.begin(true)) {
        Serial.println("Config: An error has occurred while mounting SPIFFS");
        return config;
    }

    // Read JSON controller config file.
    StaticJsonDocument<4096> doc = MCJsonConfig::ReadJsonFile(configFilePath);

    // Read controller name.
    const char *controllerName = doc["name"] | DEFAULT_CONTROLLER_NAME;
    config->ControllerName = controllerName;
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
        const bool isInverted = espPinConfig["inverted"] | false;
        const std::string attachedDevice = espPinConfig["attachedDevice"] | "nothing";

        MCChannel *espChannel = new MCChannel(ChannelType::EspPinChannel, address);
        config->EspPins.push_back(new MCChannelConfig(espChannel, pinPwrIncStep, pinPwrDecStep, isInverted, deviceTypeMap()[attachedDevice]));
    }
    log4MC::vlogf(LOG_INFO, "Config: Read ESP pin configuration (%u).", config->EspPins.size());

    // Read loco configs.
    JsonArray locoConfigs = doc["locoConfigs"].as<JsonArray>();
    for (int i = 0; i < locoConfigs.size(); i++) {
        const std::string locoConfigFile = locoConfigs[i];

        // Read JSON controller config file.
        DynamicJsonDocument locoConfigDoc = MCJsonConfig::ReadJsonFile(locoConfigFile.c_str());
        JsonObject locoConfig = locoConfigDoc.as<JsonObject>();

        // Read if loco is enabled.
        const bool enabled = locoConfig["enabled"] | true;
        if (!enabled) {
            // Skip if loco is not enabled.
            continue;
        }

        config->Locomotives.push_back(BLELocomotiveDeserializer::Deserialize(locoConfig, config->EspPins, pwrIncStep, pwrDecStep));
    }

    // Return MTC4BTConfiguration object.
    return config;
}