#include "BLEHubConfiguration.h"

BLEHubConfiguration::BLEHubConfiguration(BLEHubType hubType, std::string deviceAddress, std::vector<PortConfiguration *> channels, int16_t lightPerc, bool autoLightsOnEnabled, bool enabled)
{
    HubType = hubType;
    DeviceAddress = new NimBLEAddress(deviceAddress);
    Channels = channels;
    LightPerc = lightPerc;
    AutoLightsEnabled = autoLightsOnEnabled;
    Enabled = enabled;
}