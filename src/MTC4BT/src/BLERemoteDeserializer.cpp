#include "BLERemoteDeserializer.h"
#include "BLEHubChannel.h"
#include "MCLocoAction.h"
#include "enums.h"

typedef enum {
    LocoType = 0,
    SwitchType,
    SignalType
} itemTypes;

typedef enum {
    noMode = 0,
    listMode = 1,
    freeMode
} remoteModes;

// Map remote modes
// only loco-list, accessory-list and free are allowed
struct remoteModeMap : public std::map<std::string, remoteModes> {
    remoteModeMap()
    {
        this->operator[]("list") = listMode;
        this->operator[]("free") = freeMode;
    };
    ~remoteModeMap() {}
};

BLERemoteConfiguration *BLERemoteDeserializer::Deserialize(JsonObject remoteConfig, int16_t defaultPwrIncStep, int16_t defaultPwrDecStep)
{
    int16_t remotePwrIncStep = remoteConfig["pwrIncStep"] | defaultPwrIncStep;
    int16_t remotePwrDecStep = remoteConfig["pwrDecStep"] | defaultPwrDecStep;

    // Iterate over hub configs and copy values from the JsonDocument to BLEHubConfiguration objects.
    std::vector<BLEHubConfiguration *> hubs;
    JsonArray hubConfigs = remoteConfig["bleHubs"].as<JsonArray>();

    for (JsonObject hubConfig : hubConfigs) {
        // Read hub specific properties.
        const std::string hubType = hubConfig["type"];
        const std::string address = hubConfig["address"];
        const std::string modeString = hubConfig["mode"];

        if (remoteModeMap().count("modeString") ==  0) {
            log4MC::vlogf(LOG_ERR, "RemoteConfig: unknown mode '%s'", modeString.c_str());
            return NULL;
        }
        remoteModes mode = remoteModeMap()[modeString];

        int16_t hubPwrIncStep = hubConfig["pwrIncStep"] | remotePwrIncStep;
        int16_t hubPwrDecStep = hubConfig["pwrDecStep"] | remotePwrDecStep;

        switch (mode) {
        case listMode:
            JsonArray locosConfigs = hubConfig["list"].as<JsonArray>();
            /*
                get colours from enum.h

                "id": "FLUX",
                "color": "yellow"
            */
            for (JsonObject locoConfig : locosConfigs) {
                const std::string locoId = locoConfig["id"];
                const std::string ledColourStr = locoConfig["color"];
                const itemTypes type = itemTypes::LocoType;
                HubLedColor LedColour = HubLedColor::WHITE;
                if (hubLedColorMap()[ledColourStr] == 0) {
                     log4MC::vlogf(LOG_ERR, "RemoteConfig: unknown ledcolour '%s' using WHITE", ledColourStr.c_str());  
                }
                LedColour = hubLedColorMap()[ledColourStr];
             }
            break;
/*            
        case accessoryMode:
            break;
        case freeMode:
            break;
            */
        }

        remoteAddress PUremoteAddress; // only valid for PURemotes

        std::vector<MCChannelConfig *> channels;
        if (strcmp(hubType.c_str(), "PUController") == 0) {
            // ignore the channels for the controller, just add the led
            MCChannel *hubChannel = new MCChannel(ChannelType::BleHubChannel, "LED");
            hubChannel->SetParentAddress(address);
            std::string attachedDevice = "light";
            channels.push_back(new MCChannelConfig(hubChannel, hubPwrIncStep, hubPwrDecStep, false, 100, deviceTypeMap()[attachedDevice]));
            JsonObject remoteRanges = hubConfig["range"].as<JsonObject>();
            int max, min, portA, portB;
            min = remoteRanges["min"] | -1;
            max = remoteRanges["max"] | -1;
            portA = remoteRanges["portA"] | -1;
            portB = remoteRanges["portB"] | -1;
            if (portA == -1 || portB == -1) {
                PUremoteAddress.isRange = true; // we assume we have a min and a max
                PUremoteAddress.addr.R.min = min;
                PUremoteAddress.addr.R.max = max;
            } else {
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
        hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, buwizzPowerMap()["normal"], PUremoteAddress));
    }

    BLERemoteConfiguration *remote = new BLERemoteConfiguration(hubs);

    return remote;
}