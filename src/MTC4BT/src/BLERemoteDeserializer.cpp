#include "BLERemoteDeserializer.h"
#include "BLEHubChannel.h"
#include "MCLocoAction.h"
#include "PUremoteButtons/PUButtons.h"
#include "enums.h"
#include "rocrailitems/RRtypes.h"

typedef enum {
    LocoType = 1,
    SwitchType,
    SignalType
} itemTypes;

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

    log4MC::debug("Starting reading remotes json");

    // Iterate over hub configs and copy values from the JsonDocument to BLEHubConfiguration objects.
    std::vector<BLEHubConfiguration *> hubs;
    JsonArray hubConfigs = remoteConfig["bleHubs"].as<JsonArray>();

    for (JsonObject hubConfig : hubConfigs) {
        // Read hub specific properties.
        const std::string hubType = hubConfig["type"];
        const std::string address = hubConfig["address"];
        const std::string modeString = hubConfig["mode"];

        if (remoteModeMap().count(modeString) == 0) {
            log4MC::vlogf(LOG_ERR, "RemoteConfig: unknown mode '%s'", modeString.c_str());
            continue;
        }
        remoteModes mode = remoteModeMap()[modeString];

        int16_t hubPwrIncStep = hubConfig["pwrIncStep"] | remotePwrIncStep;
        int16_t hubPwrDecStep = hubConfig["pwrDecStep"] | remotePwrDecStep;

        // ignore the channels for the controller, just add the led
        std::vector<MCChannelConfig *> channels;
        MCChannel *hubChannel = new MCChannel(ChannelType::BleHubChannel, "LED");
        hubChannel->SetParentAddress(address);
        std::string attachedDevice = "light";
        channels.push_back(new MCChannelConfig(hubChannel, hubPwrIncStep, hubPwrDecStep, false, 100, deviceTypeMap()[attachedDevice]));

        switch (mode) {
        case listMode:
            JsonArray listConfigs = hubConfig["list"].as<JsonArray>();
            std::vector<freeListItem *> freeListItems;
            for (JsonObject listConfig : listConfigs) {
                // If the item is not in the json, it returns 0
                const char *itemId = listConfig["id"];
                const int itemAddr = listConfig["addr"] | -1;
                const std::string itemTypeStr = listConfig["type"];
                const std::string ledColourStr = listConfig["color"];

                HubLedColor ledColour = HubLedColor::WHITE;
                if (hubLedColorMap()[ledColourStr] == 0) {
                    log4MC::vlogf(LOG_ERR, "RemoteConfig: Unknown ledcolour '%s' using WHITE", ledColourStr.c_str());
                }
                if (RRdeviceMap()[itemTypeStr] == 0) {
                    log4MC::vlogf(LOG_ERR, "RemoteConfig: Unknown type '%s' ignoring this item!", itemTypeStr.c_str());
                    continue;
                }
                RRdevice itemType = RRdeviceMap()[itemTypeStr];
                ledColour = hubLedColorMap()[ledColourStr];
                freeListItems.push_back(new freeListItem(itemId, itemAddr, itemType, ledColour));
            }
            PUbuttonByType *buttons = new PUbuttonByType();

            JsonArray buttonConfigs = hubConfig["buttons"].as<JsonArray>();
            for (JsonObject buttonConfig : buttonConfigs) {
                if (buttonConfig.containsKey("button") && buttonConfig.containsKey("type") && buttonConfig.containsKey("action")) {
                    std::string buttonStr = buttonConfig["button"];
                    std::string typeStr = buttonConfig["type"];
                    std::string actionStr = buttonConfig["action"];

                    // TODO: do some sanity checks, test with values that are not mapped!
                    PUbutton button = PUbuttonMap()[buttonStr];
                    RRdevice device = RRdeviceMap()[typeStr];
                    RRaction action = RRactionMap()[actionStr];
                    if (button != PUbutton::Bplus && button != PUbutton::Bmin && button != PUbutton::Bred && button != PUbutton::Green) {
                        // ignore B+ B-  Bred and Green in list mode
                        buttons->setButton(device, button, action);
                    } else {
                        log4MC::vlogf(LOG_ERR, "RemoteConfig: Redefining '%s' button not allowed: ignoring this item!", buttonStr);
                    }
                } else {
                    log4MC::vlogf(LOG_ERR, "RemoteConfig: No key 'button', ignoring!");
                }
            }
            // store buttons and freeListItems in the remote hub config and set its type to `mode`
            hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, buttons, freeListItems));
            break;
            /*
                    case freeMode:
                        break;
                        */
        }
        //hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, buttons, freeListItems));
    }

    log4MC::vlogf(LOG_DEBUG, "Config: number of remotes %d.", hubs.size());
    BLERemoteConfiguration *remote = new BLERemoteConfiguration(hubs);

    return remote;
}