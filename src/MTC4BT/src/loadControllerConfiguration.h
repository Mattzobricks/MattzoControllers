#pragma once

#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "MTC4BTConfiguration.h"

#define DEFAULT_CONTROLLER_NAME "MTC4BT"
#define DEFAULT_PWR_INC_STEP 10
#define DEFAULT_PWR_DEC_STEP 10

MTC4BTConfiguration *loadControllerConfiguration(const char *configFilePath)
{
    // New up a configation object, so we can set its properties.
    MTC4BTConfiguration *config = new MTC4BTConfiguration();

    // Initialize file system.
    if (!SPIFFS.begin(true))
    {
        Serial.println("Config: An error has occurred while mounting SPIFFS");
        return config;
    }

    // Check if config file exists.
    File file = SPIFFS.open(configFilePath);
    if (!file)
    {
        Serial.println("Config: Failed to open config file for reading");
        return config;
    }

    // Check if config file is empty.
    size_t size = file.size();
    if (!size)
    {
        file.close();
        Serial.println("Config: Config file is empty");
        return config;
    }

    // Allocate a temporary JsonDocument.
    // Don't forget to change the capacity to match your requirements!
    // Use https://arduinojson.org/v6/assistant to compute the capacity.
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
        Serial.print(F("Config: Failed to read config file: deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return config;
    }

    // Read controller name.
    const char *controllerName = doc["name"] | DEFAULT_CONTROLLER_NAME;
    config->ControllerName = controllerName;
    log4MC::vlogf(LOG_INFO, "Config: Read controller name: %s", config->ControllerName);

    int16_t pwrIncStep = doc["pwrIncStep"] | DEFAULT_PWR_INC_STEP;
    int16_t pwrDecStep = doc["pwrDecStep"] | DEFAULT_PWR_DEC_STEP;

    // Iterate over ESP pins and copy values from the JsonDocument to MCChannelConfig objects.
    JsonArray espPinConfigs = doc["espPins"].as<JsonArray>();
    for (JsonObject espPinConfig : espPinConfigs)
    {
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

    // Iterate over loco configs and copy values from the JsonDocument to BLELocomotiveConfiguration objects.
    JsonArray locoConfigs = doc["locos"].as<JsonArray>();
    for (JsonObject locoConfig : locoConfigs)
    {
        // Read if loco is enabled.
        const bool enabled = locoConfig["enabled"] | true;
        if (!enabled)
        {
            // Skip if loco is not enabled.
            continue;
        }

        // Read loco properties.
        const uint address = locoConfig["address"];
        const std::string name = locoConfig["name"]; // | "loco_" + locoConfig["address"];
        int16_t locoPwrIncStep = locoConfig["pwrIncStep"] | pwrIncStep;
        int16_t locoPwrDecStep = locoConfig["pwrDecStep"] | pwrDecStep;

        // Iterate over hub configs and copy values from the JsonDocument to BLEHubConfiguration objects.
        std::vector<BLEHubConfiguration *> hubs;
        JsonArray hubConfigs = locoConfig["bleHubs"].as<JsonArray>();

        for (JsonObject hubConfig : hubConfigs)
        {
            // Read hub specific properties.
            const std::string hubType = hubConfig["type"];
            const std::string address = hubConfig["address"];
            int16_t hubPwrIncStep = hubConfig["pwrIncStep"] | locoPwrIncStep;
            int16_t hubPwrDecStep = hubConfig["pwrDecStep"] | locoPwrDecStep;

            // Iterate over channel configs and copy values from the JsonDocument to PortConfiguration objects.
            std::vector<MCChannelConfig *> channels;
            JsonArray channelConfigs = hubConfig["channels"].as<JsonArray>();
            for (JsonObject channelConfig : channelConfigs)
            {
                // Read hub channel properties.
                const std::string channel = channelConfig["channel"];
                const std::string attachedDevice = channelConfig["attachedDevice"] | "nothing";
                int16_t chnlPwrIncStep = channelConfig["pwrIncStep"] | hubPwrIncStep;
                int16_t chnlPwrDecStep = channelConfig["pwrDecStep"] | hubPwrDecStep;
                const char *dir = channelConfig["direction"] | "forward";
                bool isInverted = strcmp(dir, "reverse") == 0;

                MCChannel *hubChannel = new MCChannel(ChannelType::BleHubChannel, channel);
                hubChannel->SetParentAddress(address);
                channels.push_back(new MCChannelConfig(hubChannel, chnlPwrIncStep, chnlPwrDecStep, isInverted, deviceTypeMap()[attachedDevice]));
            }

            hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, enabled));
        }

        // Iterate over events and copy values from the JsonDocument to MCLocoEvent objects.
        std::vector<MCLocoEvent *> events;
        JsonArray eventConfigs = locoConfig["events"].as<JsonArray>();
        for (JsonObject eventConfig : eventConfigs)
        {
            std::vector<MCLocoTrigger *> triggers;
            JsonArray triggerConfigs = eventConfig["triggers"].as<JsonArray>();
            for (JsonObject triggerConfig : triggerConfigs)
            {
                // Read trigger properties.
                const std::string source = triggerConfig["source"] | "loco";
                const std::string eventType = triggerConfig["eventType"];
                const std::string eventId = triggerConfig["identifier"] | "";
                const std::string value = triggerConfig["value"];
                int8_t delayInMs = triggerConfig["delayInMs"] | 0;

                triggers.push_back(new MCLocoTrigger(triggerSourceMap()[source], eventType, eventId, value, delayInMs));
            }

            std::vector<MCLocoAction *> actions;
            JsonArray actionConfigs = eventConfig["actions"].as<JsonArray>();
            MCChannelConfig *foundChannel;
            for (JsonObject actionConfig : actionConfigs)
            {
                // Read action properties.
                const std::string device = actionConfig["device"] | "bleHub";
                const std::string address = actionConfig["address"] | actionConfig["pin"];
                const std::string channel = actionConfig["channel"];
                int16_t pwrPerc = actionConfig["pwrPerc"] | 0;

                foundChannel = nullptr;

                switch (channelTypeMap()[device])
                {
                case ChannelType::EspPinChannel:
                {
                    // Check if there's an ESP pin with the specified pin number in the controller config.
                    
                    for(MCChannelConfig *channelConfig : config->EspPins)
                    {
                        if (channelConfig->GetChannel()->GetAddress().compare(address) == 0)
                        {
                            foundChannel = channelConfig;
                            break;
                        }
                    }

                    if (foundChannel == nullptr)
                    {
                        log4MC::vlogf(LOG_WARNING, "Config: ESP pin %u not configured in 'espPins' section. Configured action ignored.", address);
                        continue;
                    }

                    // Check if the ESP pin with the specified address defined in the config has a light attached to it.
                    if (foundChannel->GetAttachedDeviceType() != DeviceType::Light)
                    {
                        log4MC::vlogf(LOG_WARNING, "Config: ESP pin %u in the 'espPins' section is not configured with `light` as the `attachedDevice`. Configured action ignored.", address);
                        continue;
                    }

                    break;
                }
                case ChannelType::BleHubChannel:
                {
                    BLEHubConfiguration *foundHub = nullptr;

                    if (actionConfig.containsKey("address"))
                    {
                        // Specific hub address specified, so find it.
                        for (BLEHubConfiguration *hub : hubs)
                        {
                            if (hub->DeviceAddress->toString().compare(address) == 0)
                            {
                                foundHub = hub;
                                break;
                            }
                        }
                    }
                    else
                    {
                        // No hub address specified, so assume first hub.
                        foundHub = hubs.at(0);
                    }

                    if (foundHub == nullptr)
                    {
                        log4MC::vlogf(LOG_ERR, "Config: Hub '%s' not configured in this loco's 'bleHubs' section.", address);
                    }

                    // Check if the specified channel is defined in the hub config.
                    for (MCChannelConfig *channelConfig : foundHub->Channels)
                    {
                        if (channelConfig->GetChannel()->GetAddress().compare(channel) == 0)
                        {
                            foundChannel = channelConfig;
                            break;
                        }
                    }

                    if (foundChannel == nullptr)
                    {
                        log4MC::vlogf(LOG_ERR, "Config: Hub channel %s not configured in this loco's 'bleHubs' section.", channel);
                    }

                    break;
                }
                }

                actions.push_back(new MCLocoAction(foundChannel->GetChannel(), pwrPerc));
            }

            events.push_back(new MCLocoEvent(triggers, actions));
        }

        config->Locomotives.push_back(new BLELocomotiveConfiguration(address, name, hubs, events, enabled));
    }
    log4MC::vlogf(LOG_INFO, "Config: Read loco configuration (%u).", config->Locomotives.size());

    // Close the config file.
    file.close();

    // Return MTC4BTConfiguration object.
    return config;
}