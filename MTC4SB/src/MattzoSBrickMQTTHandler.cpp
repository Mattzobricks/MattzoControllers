#include "MattzoSBrickMQTTHandler.h"

void MattzoSBrickMQTTHandler::Handle(const char *message, ulong numSBricks, SBrickHubClient *sbricks[])
{
    String strMessage = String(message);
    if (isNodeType(strMessage, "lc"))
    {
        handleLc(message, numSBricks, sbricks);
    }
}

bool MattzoSBrickMQTTHandler::isNodeType(String message, const char *nodeName)
{
    return message.substring(1, message.indexOf(" ")).equals(nodeName);
}

void MattzoSBrickMQTTHandler::handleLc(const String message, ulong numSBricks, SBrickHubClient *sbricks[])
{
    String lcId = getAttr(message, "id");
    for (int i = 0; i < numSBricks; i++)
    {
        if (sbricks[i]->getDeviceName().compare(lcId.c_str()) == 0)
        {
            // Get channel index (0=A, 1=B, 2=C, 3=D).
            String addrStr = getAttr(message, "addr");
            int addrInt = std::atoi(addrStr.c_str());
            SBrickHubChannel::SBrickChannel channel = static_cast<SBrickHubChannel::SBrickChannel>(addrInt);

            // Get direction (true=forward, false=backward).
            String dirStr = getAttr(message, "dir");
            bool dirBool = dirStr == "true";
            int8_t dirMultiplier = dirBool ? 1 : -1;

            // Get target speed (percentage of max).
            String v = getAttr(message, "V");
            int speed = std::atoi(v.c_str()) * dirMultiplier;

            // Determine target speed by taking requested percentage of max speed.
            float speedMultiplier = float(SBrickHubChannel::MAX_CHANNEL_SPEED) / 100;
            int16_t targetSpeed = speed * speedMultiplier;

            // Execute drive command.
            sbricks[i]->DriveChannel(channel, targetSpeed);
            break;
        }
    }
}

String MattzoSBrickMQTTHandler::getAttr(const String message, const String attrName)
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