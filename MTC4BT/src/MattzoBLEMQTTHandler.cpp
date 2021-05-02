#include "MattzoBLEMQTTHandler.h"

void MattzoBLEMQTTHandler::Handle(const char *message, ulong locoCount, BLELocomotive *locos[])
{
    char *pos;

    // parse the rocrail mqtt messages, all of them,
    if ((pos = strstr(message, "<sys ")) != nullptr)
    {
        // found <sys
        handleSys(pos, locoCount, locos);
    }
    else if ((pos = strstr(message, "<lc ")) != nullptr)
    {
        // found <lc
        handleLc(pos, locoCount, locos);
    }
    else if ((pos = strstr(message, "<fn ")) != nullptr)
    {
        // found <fn
        handleFn(pos, locoCount, locos);
    }
    else if ((pos = strstr(message, "<sw ")) != nullptr)
    {
        // first go for "<sw"
    }
    else if ((pos = strstr(message, "<clock ")) != nullptr)
    {
        // found clock
    } // IGNORE THE REST
}

void MattzoBLEMQTTHandler::handleSys(const char *message, ulong locoCount, BLELocomotive *locos[])
{
    char *cmd = nullptr;
    if (!XmlParser::tryReadCharAttr(message, "cmd", &cmd))
    {
        // TODO: Log error, ignore message.
        return;
    }

    if (strcmp(cmd, "ebreak") == 0 || strcmp(cmd, "stop") == 0 || strcmp(cmd, "shutdown") == 0)
    {
        // Upon receiving "stop", "ebreak" or "shutdown" system command from Rocrail, the global emergency break flag is set. Train will stop immediately.
        for (int i = 0; i < locoCount; i++)
        {
            locos[i]->EmergencyBreak(true);
        }

        return;
    }

    if (strcmp(cmd, "go") == 0)
    {
        // Upon receiving "go" command, the emergency break flag is be released (i.e. pressing the light bulb in Rocview).
        for (int i = 0; i < locoCount; i++)
        {
            locos[i]->EmergencyBreak(false);
        }

        return;
    }
}

void MattzoBLEMQTTHandler::handleLc(const char *message, ulong locoCount, BLELocomotive *locos[])
{
    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr))
    {
        // TODO: Log error, ignore message.
        return;
    }

    for (int i = 0; i < locoCount; i++)
    {
        // Find loco by address.
        if (locos[i]->GetLocoAddress() == addr)
        {
            // Get target speed.
            int speed;
            if (!XmlParser::tryReadIntAttr(message, "V", &speed))
            {
                // TODO: Log error, ignore message.
            }

            // Get min speed.
            int minSpeed;
            if (!XmlParser::tryReadIntAttr(message, "V_min", &minSpeed))
            {
                // TODO: Log error, ignore message.
            }

            if (speed != 0 && speed < minSpeed)
            {
                // Requested speed is too low, we can ignore it.
                return;
            }

            // Get max speed.
            int maxSpeed;
            if (!XmlParser::tryReadIntAttr(message, "V_max", &maxSpeed))
            {
                // TODO: Log error, ignore message.
            }

            // Get speed mode (percentage or km/h).
            char *mode;
            if (!XmlParser::tryReadCharAttr(message, "V_mode", &mode))
            {
                // TODO: Log error, ignore message.
            }

            // Get direction (true=forward, false=backward).
            bool dirBool;
            if (!XmlParser::tryReadBoolAttr(message, "dir", &dirBool))
            {
                // TODO: Log error, ignore message.
            }

            // Calculate target speed percentage (as percentage if mode is "percent", or else as a percentage of max speed).
            int targetSpeedPerc = strcmp(mode, "percent") == 0 ? speed : (speed * maxSpeed) / 100;

            // Execute drive command.
            int8_t dirMultiplier = dirBool ? 1 : -1;
            locos[i]->Drive(minSpeed, targetSpeedPerc * dirMultiplier);

            if (locos[i]->GetAutoLightsEnabled())
            {
                // TODO: Determine lights on or off based on target motor speed percentage.
                // locos[i]->SetLights(speed != 0);
            }

            return;
        }
    }
}

void MattzoBLEMQTTHandler::handleFn(const char *message, ulong locoCount, BLELocomotive *locos[])
{
    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr))
    {
        // TODO: Log error, ignore message.
        return;
    }

    for (int i = 0; i < locoCount; i++)
    {
        // Find loco by address.
        if (locos[i]->GetLocoAddress() == addr)
        {
            // if (locos[i]->GetAutoLightsEnabled())
            // {
            //     // Found the loco the command is for, but it uses automatic lights (on while driving). Ignore command.
            //     return;
            // }

            // Get channel index (f0=A, f1=B, f2=C, f3=D).
            int fnchanged;
            if (!XmlParser::tryReadIntAttr(message, "fnchanged", &fnchanged))
            {
                // TODO: Log error, ignore message.
            }

            // Query fnchangedstate attribute. This is the new state of the lights (true=on, false=off) of a channel.
            bool fnchangedstate;
            if (!XmlParser::tryReadBoolAttr(message, "fnchangedstate", &fnchangedstate))
            {
                // TODO: Log error, ignore message.
            }

            // Determine actual channel.
            // HubChannel channel = static_cast<HubChannel>(fnchanged);

            // TODO: Turn lights on for the requested channel (if channel has lights attached!).
            // locos[i]->SetLights(channel, fnchangedstate);
        }
    }
}