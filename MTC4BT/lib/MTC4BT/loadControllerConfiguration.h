#pragma once

#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "MTC4BTConfiguration.h"

#define DEFAULT_CONTROLLER_NAME "MTC4BT"

MTC4BTConfiguration *loadControllerConfiguration(const char *configFilePath)
{
    // New up a configation object, so we can set its properties.
    MTC4BTConfiguration *config = new MTC4BTConfiguration();

    // Initialize file system.
    if (!SPIFFS.begin(true))
    {
        Serial.println("An error has occurred while mounting SPIFFS");
        return config;
    }

    // Check if config file exists.
    File file = SPIFFS.open(configFilePath);
    if (!file)
    {
        Serial.println("Failed to open config file for reading");
        return config;
    }

    // Check if config file is empty.
    size_t size = file.size();
    if (!size)
    {
        file.close();
        Serial.println("Config file is empty");
        return config;
    }

    // Allocate a temporary JsonDocument.
    // Don't forget to change the capacity to match your requirements!
    // Use arduinojson.org/v6/assistant to compute the capacity.
    // Use the settings:
    // - Processor: ESP32
    // - Mode: Deserialize
    // - Input type: Stream
    // Including some slack (1024) in case the strings change, and rounded to a power of two.
    StaticJsonDocument<4096> doc;

    // Deserialize the JSON document.
    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
        Serial.print(F("Failed to read config file: deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return config;
    }

    // Read controller name.
    const char *controllerName = doc["name"] | DEFAULT_CONTROLLER_NAME;
    config->ControllerName = controllerName;
    log4MC::vlogf(LOG_INFO, "Controller name: %s", config->ControllerName);

    // Iterate over loco configs and copy values from the JsonDocument to BLELocomotiveConfiguration objects.
    JsonArray locoConfigs = doc["locos"].as<JsonArray>();
    for (JsonObject locoConfig : locoConfigs)
    {
        // Read loco properties.
        const uint address = locoConfig["address"];
        const std::string name = locoConfig["name"];
        const int16_t lightPerc = locoConfig["lightPerc"] | 100;
        const bool autoLightsOnEnabled = locoConfig["autoLightsOnEnabled"] | false;
        const bool enabled = locoConfig["enabled"] | true;

        // Iterate over hub configs and copy values from the JsonDocument to BLEHubConfiguration objects.
        std::vector<BLEHubConfiguration *> hubs;
        JsonArray hubConfigs = locoConfig["bleHubs"].as<JsonArray>();

        for (JsonObject hubConfig : hubConfigs)
        {
            // Read hub specific properties.
            const std::string hubType = hubConfig["type"];
            const std::string address = hubConfig["address"];

            // Iterate over channel configs and copy values from the JsonDocument to ChannelConfiguration objects.
            std::vector<ChannelConfiguration *> channels;
            JsonArray channelConfigs = hubConfig["channels"].as<JsonArray>();
            for (JsonObject channelConfig : channelConfigs)
            {
                // Read hub channel properties.
                std::string channel = channelConfig["channel"];
                std::string attachedDevice = channelConfig["attachedDevice"] | "nothing";
                int16_t speedStep = channelConfig["speedStep"] | 10;
                int16_t breakStep = channelConfig["breakStep"] | 20;
                std::string dir = channelConfig["direction"] | "forward";

                channels.push_back(new ChannelConfiguration(hubChannelMap()[channel], attachedDeviceMap()[attachedDevice], speedStep, breakStep, hubChannelDirectionMap()[dir]));
            }

            hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, lightPerc, autoLightsOnEnabled, enabled));
        }

        BLELocomotive *loco = new BLELocomotive(new BLELocomotiveConfiguration(address, name, hubs, lightPerc, autoLightsOnEnabled, enabled));
        config->Locomotives.push_back(loco);
    }

    // Close the config file.
    file.close();

    // Configuration debug info:
    // Serial.print("Nr of loco's found in config: ");
    // Serial.println(config->Locomotives.size());
    // Serial.print("Enabled first loco: ");
    // Serial.println(config->Locomotives.at(0)->IsEnabled());
    // Serial.print("Name of first loco: ");
    // Serial.println(config->Locomotives.at(0)->GetLocoName().c_str());
    // Serial.print("Address of first loco: ");
    // Serial.println(config->Locomotives.at(0)->GetLocoAddress());
    // Serial.print("Nr of hubs in first loco: ");
    // Serial.println(config->Locomotives.at(0)->GetHubCount());
    // Serial.print("Auto-lights on enabled for first hub in first loco: ");
    // Serial.println(config->Locomotives.at(0)->GetHub(0)->GetAutoLightsEnabled());
    // Serial.print("Enabled for first hub in first loco: ");
    // Serial.println(config->Locomotives.at(0)->GetHub(0)->IsEnabled());
    // Serial.print("Enabled second loco: ");
    // Serial.println(config->Locomotives.at(1)->IsEnabled());
    // Serial.print("Auto-lights on enabled for first hub in second loco: ");
    // Serial.println(config->Locomotives.at(1)->GetHub(0)->GetAutoLightsEnabled());

    // Return MTC4BTConfiguration object.
    return config;
}