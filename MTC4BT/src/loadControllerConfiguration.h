#pragma once

#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "MTC4BTConfiguration.h"
#include "Fn.h"

#define DEFAULT_CONTROLLER_NAME "MTC4BT"

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
        Serial.print(F("Config: Failed to read config file: deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return config;
    }

    // Read controller name.
    const char *controllerName = doc["name"] | DEFAULT_CONTROLLER_NAME;
    config->ControllerName = controllerName;
    log4MC::vlogf(LOG_INFO, "Config: Read controller name: %s", config->ControllerName);

    // Iterate over ESP pins and copy values from the JsonDocument to DeviceConfiguration objects.
    JsonArray espPinConfigs = doc["espPins"].as<JsonArray>();
    for (JsonObject espPinConfig : espPinConfigs)
    {
        // Use pin number as its device address.
        const std::string address = espPinConfig["pin"];
        const bool inverted = espPinConfig["inverted"] | false;
        const std::string attachedDevice = espPinConfig["attachedDevice"] | "nothing";

        config->EspPins.push_back(new DeviceConfiguration(HardwareType::EspPin, address, inverted, attachedDeviceMap()[attachedDevice]));
    }
    log4MC::vlogf(LOG_INFO, "Config: Read ESP pin configuration (%u).", config->EspPins.size());

    // Iterate over functions and copy values from the JsonDocument to Fn objects.
    JsonArray fnConfigs = doc["fn"].as<JsonArray>();
    for (JsonObject fnConfig : fnConfigs)
    {
        const char *fnName = fnConfig["name"];
        const int pin = fnConfig["pin"];

        // Check if there's an ESP pin with the specified address defined in the config.
        DeviceConfiguration *fnDevice = nullptr;
        for (int i = 0; i < config->EspPins.size(); i++)
        {
            DeviceConfiguration *espPin = config->EspPins.at(i);
            if (espPin->GetAddressAsEspPinNumber() == pin)
            {
                fnDevice = espPin;
                break;
            }
        }

        if (fnDevice == nullptr)
        {
            log4MC::vlogf(LOG_ERR, "Config: ESP pin %u not configured in 'espPins' section.", pin);
        }

        // Check if the ESP pin with the specified address defined in the config has a light attached to it.
        if (fnDevice->GetAttachedDeviceType() != AttachedDevice::LIGHT)
        {
            log4MC::vlogf(LOG_ERR, "Config: ESP pin %u in the 'espPins' section is not configured with `light` as the `attachedDevice`.", pin);
        }

        config->Functions.push_back(new Fn(functionMap()[fnName], fnDevice));
    }
    log4MC::vlogf(LOG_INFO, "Config: Read function configuration (%u).", config->Functions.size());

    // Iterate over loco configs and copy values from the JsonDocument to BLELocomotiveConfiguration objects.
    JsonArray locoConfigs = doc["locos"].as<JsonArray>();
    for (JsonObject locoConfig : locoConfigs)
    {
        // Read loco properties.
        const uint address = locoConfig["address"];
        const std::string name = locoConfig["name"]; // | "loco_" + locoConfig["address"];
        int16_t speedStep = locoConfig["speedStep"] | 10;
        int16_t brakeStep = locoConfig["brakeStep"] | 20;
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

            // Iterate over channel configs and copy values from the JsonDocument to DeviceConfiguration objects.
            std::vector<DeviceConfiguration *> channels;
            JsonArray channelConfigs = hubConfig["channels"].as<JsonArray>();
            for (JsonObject channelConfig : channelConfigs)
            {
                // Read hub channel properties.
                const std::string channel = channelConfig["channel"];
                const std::string attachedDevice = channelConfig["attachedDevice"] | "nothing";
                const char *dir = channelConfig["direction"] | "forward";
                bool isInverted = strcmp(dir, "reverse") == 0;

                channels.push_back(new DeviceConfiguration(HardwareType::BleHub, channel, isInverted, attachedDeviceMap()[attachedDevice]));
            }

            hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, lightPerc, autoLightsOnEnabled, enabled));
        }

        // Iterate over functions and copy values from the JsonDocument to Fn objects.
        std::vector<Fn *> functions;
        JsonArray fnConfigs = locoConfig["fn"].as<JsonArray>();
        for (JsonObject fnConfig : fnConfigs)
        {
            const char *fnName = fnConfig["name"];
            const std::string device = fnConfig["device"];

            DeviceConfiguration *fnDevice = nullptr;
            DeviceConfiguration *tmpDevice;

            switch (hardwareTypeMap()[device])
            {
            case HardwareType::EspPin:
            {
                // Check if there's an ESP pin with the specified pin number in the config.
                const int pin = fnConfig["pin"];

                for (int i = 0; i < config->EspPins.size(); i++)
                {
                    tmpDevice = config->EspPins.at(i);
                    if (tmpDevice->GetAddressAsEspPinNumber() == pin)
                    {
                        fnDevice = tmpDevice;
                        break;
                    }
                }

                if (fnDevice == nullptr)
                {
                    log4MC::vlogf(LOG_ERR, "Config: ESP pin %u not configured in 'espPins' section.", pin);
                }

                // Check if the ESP pin with the specified address defined in the config has a light attached to it.
                if (fnDevice->GetAttachedDeviceType() != AttachedDevice::LIGHT)
                {
                    log4MC::vlogf(LOG_ERR, "Config: ESP pin %u in the 'espPins' section is not configured with `light` as the `attachedDevice`.", pin);
                }

                break;
            }
            case HardwareType::BleHub:
            {
                // Check if there's a hub with the specified address in the config.
                const std::string address = fnConfig["address"];
                const std::string channel = fnConfig["channel"];

                std::string hubAddress;
                for (BLEHubConfiguration *hub : hubs)
                {
                    if (hub->DeviceAddress->toString().compare(address) == 0)
                    {
                        // Keep hub address for reference.
                        hubAddress = hub->DeviceAddress->toString();

                        // Check if the specified channel is defined in the hub config.
                        for (int i = 0; i < hub->Channels.size(); i++)
                        {
                            tmpDevice = hub->Channels.at(i);
                            if (tmpDevice->GetAddress().compare(channel) == 0)
                            {
                                fnDevice = tmpDevice;
                                break;
                            }
                        }

                        break;
                    }
                }

                if (fnDevice == nullptr)
                {
                    log4MC::vlogf(LOG_ERR, "Config: Hub '%s' or channel %s not configured in this loco's 'bleHubs' section.", address, channel);
                }

                // Check if the given channel of the hub with the specified address defined in the config has a light attached to it.
                if (fnDevice->GetAttachedDeviceType() != AttachedDevice::LIGHT)
                {
                    log4MC::vlogf(LOG_ERR, "Config: Channel %s of hub '%s' in the 'bleHubs' section is not configured with `light` as the `attachedDevice`.", channel, address);
                }

                // Keep hub address for reference.
                fnDevice->SetParentAddress(address);

                break;
            }
            }

            functions.push_back(new Fn(functionMap()[fnName], fnDevice));
        }
        log4MC::vlogf(LOG_INFO, "Config: Read function configuration (%u).", functions.size());

        BLELocomotive *loco = new BLELocomotive(new BLELocomotiveConfiguration(address, name, hubs, functions, speedStep, brakeStep, lightPerc, autoLightsOnEnabled, enabled));
        config->Locomotives.push_back(loco);
    }
    log4MC::vlogf(LOG_INFO, "Config: Read loco configuration (%u).", config->Locomotives.size());

    // Close the config file.
    file.close();

    // Return MTC4BTConfiguration object.
    return config;
}