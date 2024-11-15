#include "BLELocomotiveDeserializer.h"
#include "BLEHubChannel.h"
#include "MCLocoAction.h"

BLELocomotiveConfiguration *BLELocomotiveDeserializer::Deserialize(JsonObject locoConfig, std::vector<MCChannelConfig *> espPins, int16_t defaultPwrIncStep, int16_t defaultPwrDecStep)
{
    // Read loco properties.
    const uint address = locoConfig["address"];
    const std::string name = locoConfig["name"]; // | "loco_" + locoConfig["address"];
    int16_t locoPwrIncStep = locoConfig["pwrIncStep"] | defaultPwrIncStep;
    int16_t locoPwrDecStep = locoConfig["pwrDecStep"] | defaultPwrDecStep;

    // Iterate over hub configs and copy values from the JsonDocument to BLEHubConfiguration objects.
    std::vector<BLEHubConfiguration *> hubs;
    JsonArray hubConfigs = locoConfig["bleHubs"].as<JsonArray>();

    for (JsonObject hubConfig : hubConfigs) {
        // Read hub specific properties.
        const std::string hubType = hubConfig["type"];
        const std::string address = hubConfig["address"];
        int16_t hubPwrIncStep = hubConfig["pwrIncStep"] | locoPwrIncStep;
        int16_t hubPwrDecStep = hubConfig["pwrDecStep"] | locoPwrDecStep;
        const std::string powerlevel = hubConfig["powerlevel"] | "normal"; // for Buwizz2 only, default is 2
        remoteAddress PUremoteAddress;                                     // only valid for PURemotes

        std::vector<MCChannelConfig *> channels;
        if (strcmp(hubType.c_str(), "PUController") == 0) {
            // ignore the channels for the controller, just add the led
            MCChannel *hubChannel = new MCChannel(ChannelType::BleHubChannel, "LED");
            hubChannel->SetParentAddress(address);
            std::string attachedDevice = "light";
            channels.push_back(new MCChannelConfig(hubChannel, hubPwrIncStep, hubPwrDecStep, false, deviceTypeMap()[attachedDevice]));
            JsonObject remoteRanges = hubConfig["range"].as<JsonObject>();
            int max,min, portA, portB;
            min = remoteRanges["min"] | -1;
            max = remoteRanges["max"] | -1;
            portA = remoteRanges["portA"] | -1;
            portB = remoteRanges["portB"] | -1;
            if (portA == -1 || portB == -1) {
                PUremoteAddress.isRange = true; // we assume we have a min and a max
                PUremoteAddress.addr.R.min = min;
                PUremoteAddress.addr.R.max = max;
            }else {
                PUremoteAddress.isRange = false;
                PUremoteAddress.addr.F.portA = portA;
                PUremoteAddress.addr.F.portB = portB;
            }
        } else {
            // Iterate over channel configs and copy values from the JsonDocument to PortConfiguration objects.
            JsonArray channelConfigs = hubConfig["channels"].as<JsonArray>();
            for (JsonObject channelConfig : channelConfigs) {
                // Read hub channel properties.
                const std::string channel = channelConfig["channel"];
                std::string attachedDevice = channelConfig["attachedDevice"] | "nothing";
                const int16_t chnlPwrIncStep = channelConfig["pwrIncStep"] | hubPwrIncStep;
                const int16_t chnlPwrDecStep = channelConfig["pwrDecStep"] | hubPwrDecStep;
                int16_t chnlPwr = channelConfig["power"] | 100;
            if (chnlPwr < 1 || chnlPwr > 100) {
                log4MC::vlogf(LOG_ERR, "Config: ERROR the 'power' value must be between 1 and 100, using 100!");
                chnlPwr = 100;
            }
            const char *dir = channelConfig["direction"] | "forward";
                bool isInverted = strcmp(dir, "backward") == 0 || strcmp(dir, "reverse") == 0;
                bool isPU = strcmp(hubType.c_str(), "PU") == 0;

                MCChannel *hubChannel = new MCChannel(ChannelType::BleHubChannel, channel);
                hubChannel->SetParentAddress(address);

                if (bleHubChannelMap()[hubChannel->GetAddress()] == BLEHubChannel::OnboardLED) {
                    if (!isPU) {
                        // We currently only support the onboad LED of the PU Hub, so we skip this LED channel for now.
                        log4MC::vlogf(LOG_WARNING, "Config: Support for hub channel %s is currently only available for PU Hubs.", channel);
                        continue;
                    }

                    // Enforce have a 'light' device "attached" for channel of type 'LED', no matter what the config said.
                    attachedDevice = "light";
                }

                channels.push_back(new MCChannelConfig(hubChannel, chnlPwrIncStep, chnlPwrDecStep, isInverted, chnlPwr, deviceTypeMap()[attachedDevice]));
            }
        }
        hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, buwizzPowerMap()[powerlevel],PUremoteAddress));
    }

    // Iterate over events and copy values from the JsonDocument to MCLocoEvent objects.
    std::vector<MCLocoEvent *> events;
    JsonArray eventConfigs = locoConfig["events"].as<JsonArray>();
    for (JsonObject eventConfig : eventConfigs) {
        std::vector<MCLocoTrigger *> triggers;
        JsonArray triggerConfigs = eventConfig["triggers"].as<JsonArray>();
        for (JsonObject triggerConfig : triggerConfigs) {
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
        for (JsonObject actionConfig : actionConfigs) {
            // Read action properties.
            const std::string device = actionConfig["device"] | "bleHub";
            const std::string address = actionConfig["address"] | actionConfig["pin"];
            const std::string channel = actionConfig["channel"];
            int16_t pwrPerc = actionConfig["pwrPerc"] | 0;
            const std::string color = actionConfig["color"] | "";

            foundChannel = nullptr;

            switch (channelTypeMap()[device]) {
            case ChannelType::EspPinChannel: {
                // Check if there's an ESP pin with the specified pin number in the controller config.

                for (MCChannelConfig *channelConfig : espPins) {
                    if (channelConfig->GetChannel()->GetAddress().compare(address) == 0) {
                        foundChannel = channelConfig;
                        break;
                    }
                }

                if (foundChannel == nullptr) {
                    log4MC::vlogf(LOG_WARNING, "Config: ESP pin %u not configured in 'espPins' section. Configured action ignored.", address);
                    continue;
                }

                // Check if the ESP pin with the specified address defined in the config has a light attached to it.
                if (foundChannel->GetAttachedDeviceType() != DeviceType::Light) {
                    log4MC::vlogf(LOG_WARNING, "Config: ESP pin %u in the 'espPins' section is not configured with `light` as the `attachedDevice`. Configured action ignored.", address);
                    continue;
                }

                break;
            }
            case ChannelType::BleHubChannel: {
                BLEHubConfiguration *foundHub = nullptr;

                if (actionConfig.containsKey("address")) {
                    // Specific hub address specified, so find it.
                    for (BLEHubConfiguration *hub : hubs) {
                        if (hub->DeviceAddress->toString().compare(address) == 0) {
                            foundHub = hub;
                            break;
                        }
                    }
                } else {
                    // No hub address specified, so assume first hub.
                    foundHub = hubs.at(0);
                }

                if (foundHub == nullptr) {
                    log4MC::vlogf(LOG_ERR, "Config: Hub '%s' not configured in this loco's 'bleHubs' section.", address);
                }

                // Check if the specified channel is defined in the hub config.
                for (MCChannelConfig *channelConfig : foundHub->Channels) {
                    if (channelConfig->GetChannel()->GetAddress().compare(channel) == 0) {
                        foundChannel = channelConfig;
                        break;
                    }
                }

                if (foundChannel == nullptr) {
                    log4MC::vlogf(LOG_ERR, "Config: Hub channel %s not configured in this loco's 'bleHubs' section.", channel);
                }

                break;
            }
            }

            actions.push_back(new MCLocoAction(foundChannel->GetChannel(), pwrPerc, hubLedColorMap()[color]));
        }

        events.push_back(new MCLocoEvent(triggers, actions));
    }

    BLELocomotiveConfiguration *loco = new BLELocomotiveConfiguration(address, name, hubs, events);

    return loco;
}