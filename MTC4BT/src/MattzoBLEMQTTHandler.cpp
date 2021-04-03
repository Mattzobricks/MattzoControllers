#include "MattzoBLEMQTTHandler.h"

void MattzoBLEMQTTHandler::Handle(const char *message, ulong hubCount, BLEHub *hubs[])
{
    String strMessage = String(message);

    if (isNodeType(strMessage, "sys"))
    {
        handleSys(message, hubCount, hubs);
        return;
    }

    if (isNodeType(strMessage, "lc"))
    {
        handleLc(message, hubCount, hubs);
        return;
    }

    if (isNodeType(strMessage, "fn"))
    {
        handleFn(message, hubCount, hubs);
        return;
    }
}

bool MattzoBLEMQTTHandler::isNodeType(String message, const char *nodeName)
{
    return message.substring(1, message.indexOf(" ")).equals(nodeName);
}

void MattzoBLEMQTTHandler::handleSys(const String message, ulong hubCount, BLEHub *hubs[])
{
    String cmd = getAttr(message, "cmd");
    if (cmd.equals("ebreak") || cmd.equals("stop") || cmd.equals("shutdown"))
    {
        // Upon receiving "stop", "ebreak" or "shutdown" system command from Rocrail, the global emergency break flag is set. Train will stop immediately.
        for (int i = 0; i < hubCount; i++)
        {
            hubs[i]->EmergencyBreak(true);
        }

        return;
    }

    if (cmd.equals("go"))
    {
        // Upon receiving "go" command, the emergency break flag is be released (i.e. pressing the light bulb in Rocview).
        for (int i = 0; i < hubCount; i++)
        {
            hubs[i]->EmergencyBreak(false);
        }

        return;
    }
}

void MattzoBLEMQTTHandler::handleLc(const String message, ulong hubCount, BLEHub *hubs[])
{
    String lcId = getAttr(message, "id");
    for (int i = 0; i < hubCount; i++)
    {
        if (hubs[i]->GetDeviceName().compare(lcId.c_str()) == 0)
        {
            // Get channel index (0=A, 1=B, 2=C, 3=D).
            // String addrStr = getAttr(message, "addr");
            // int motorChannelIndex = std::atoi(addrStr.c_str());
            // HubChannel motorChannel = static_cast<HubChannel>(motorChannelIndex);

            // Get direction (true=forward, false=backward).
            String dirStr = getAttr(message, "dir");
            bool dirBool = dirStr == "true";
            int8_t dirMultiplier = dirBool ? 1 : -1;

            // Get target speed (percentage of max).
            String v = getAttr(message, "V");
            int speedPerc = std::atoi(v.c_str()) * dirMultiplier;

            // Execute drive command.
            hubs[i]->Drive(speedPerc);

            if (hubs[i]->GetAutoLightsEnabled())
            {
                // Determine lights on or off based on target motor speed percentage.
                hubs[i]->SetLights(speedPerc != 0);
            }

            return;
        }
    }
}

void MattzoBLEMQTTHandler::handleFn(const String message, ulong numhubs, BLEHub *hubs[])
{
    String lcId = getAttr(message, "id");
    for (int i = 0; i < numhubs; i++)
    {
        if (hubs[i]->GetDeviceName().compare(lcId.c_str()) == 0)
        {
            // if (hubs[i]->GetAutoLightsEnabled())
            // {
            //     // Found the loco the command is for, but it uses automatic lights (on while driving). Ignore command.
            //     return;
            // }

            // Get channel index (f0=A, f1=B, f2=C, f3=D).
            String fnChanged = getAttr(message, "fnchanged");
            int channelIndex = std::atoi(fnChanged.c_str());
            HubChannel channel = static_cast<HubChannel>(channelIndex);

            // Query fx attribute. This is the state of the lights (true=on, false=off) of a channel.
            String attr = "f" + String(channelIndex);
            bool lightsOn = getAttr(message, attr) == "true";

            // Turn lights on for the requested channel (if channel has lights attached!).
            hubs[i]->SetLights(channel, lightsOn);
        }
    }
}

String MattzoBLEMQTTHandler::getAttr(const String message, const String attrName)
{
    int beginIndex = message.indexOf(" " + attrName + "=\"") + attrName.length() + 3;
    int endIndex = message.indexOf("\" ", beginIndex);

    if (endIndex == -1)
    {
        // Maybe it's the last attribute, with a forward slash '/' after the double quote, instead of a space ' '?
        endIndex = message.indexOf("\"/", beginIndex);
    }

    return message.substring(beginIndex, endIndex);
}