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
    log4MC::debug("Starting reading remotes json");

    // Iterate over hub configs and copy values from the JsonDocument to BLEHubConfiguration objects.
    std::vector<BLEHubConfiguration *> hubs;

    //  Read hub specific properties.
    const std::string hubType = remoteConfig["type"];
    const std::string address = remoteConfig["address"];
    const std::string modeString = remoteConfig["mode"];

    if (remoteModeMap().count(modeString) == 0) {
        log4MC::vlogf(LOG_ERR, "RemoteConfig: unknown mode '%s'", modeString.c_str());
        return NULL;
    }
    remoteModes mode = remoteModeMap()[modeString];

    int16_t hubPwrIncStep = remoteConfig["pwrIncStep"] | defaultPwrIncStep;
    int16_t hubPwrDecStep = remoteConfig["pwrDecStep"] | defaultPwrDecStep;

    // ignore the channels for the controller, just add the led
    std::vector<MCChannelConfig *> channels;
    MCChannel *hubChannel = new MCChannel(ChannelType::BleHubChannel, "LED");
    hubChannel->SetParentAddress(address);
    std::string attachedDevice = "light";
    channels.push_back(new MCChannelConfig(hubChannel, hubPwrIncStep, hubPwrDecStep, false, 100, deviceTypeMap()[attachedDevice]));

    JsonArray freeConfigs;
    JsonArray buttonConfigs;
    JsonArray listConfigs;
    std::vector<freeListItem *> freeListItems;
    PUbuttonByType *buttons = new PUbuttonByType();
    PUbuttonList *buttonList = new PUbuttonList();

    switch (mode) {
    case listMode:
        listConfigs = remoteConfig["list"].as<JsonArray>();

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

        buttonConfigs = remoteConfig["buttons"].as<JsonArray>();
        for (JsonObject buttonConfig : buttonConfigs) {
            if (buttonConfig.containsKey("button") && buttonConfig.containsKey("type") && buttonConfig.containsKey("action")) {
                std::string buttonStr = buttonConfig["button"];
                std::string typeStr = buttonConfig["type"];
                std::string actionStr = buttonConfig["action"];

                // WONTDO: leave it as an undocumented feature: Some sanity checks, test with values that are not mapped!
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

    case freeMode:
        JsonArray freeConfigs = remoteConfig["buttons"].as<JsonArray>();
        for (JsonObject freeConfig : freeConfigs) {
            if (freeConfig.containsKey("button") && freeConfig.containsKey("type") && freeConfig.containsKey("action") && (freeConfig.containsKey("id") || freeConfig.containsKey("addr"))) {
                std::string buttonStr = freeConfig["button"];
                std::string typeStr = freeConfig["type"];
                std::string actionStr = freeConfig["action"];
                const char *itemId = freeConfig["id"];
                const int itemAddr = freeConfig["addr"] | -1;

                // TODO: do some sanity checks, test with values that are not mapped!
                PUbutton button = PUbuttonMap()[buttonStr];
                RRdevice device = RRdeviceMap()[typeStr];
                RRaction action = RRactionMap()[actionStr];

                buttonList->addButtonItem(button, new freeButtonItem(itemId, itemAddr, device, action));
            } else {
                log4MC::vlogf(LOG_ERR, "RemoteConfig: No valid 'button' found, ignoring!");
            }
        }
        // store buttons and freeListItems in the remote hub config and set its type to `mode`
        hubs.push_back(new BLEHubConfiguration(bleHubTypeMap()[hubType], address, channels, buttonList));
        break;
    }

    log4MC::vlogf(LOG_DEBUG, "Config: number of remotes %d.", hubs.size());
    BLERemoteConfiguration *remote = new BLERemoteConfiguration(hubs);

    return remote;
}