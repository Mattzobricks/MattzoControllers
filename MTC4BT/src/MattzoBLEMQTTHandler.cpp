#include "MattzoBLEMQTTHandler.h"
#include "log4MC.h"

void MattzoBLEMQTTHandler::Handle(const char *message, std::vector<BLELocomotive *> locos)
{
    char *pos;

    // parse the rocrail mqtt messages, all of them,
    if ((pos = strstr(message, "<sys ")) != nullptr)
    {
        // found <sys
        handleSys(pos, locos);
    }
    else if ((pos = strstr(message, "<lc ")) != nullptr)
    {
        // found <lc
        handleLc(pos, locos);
    }
    else if ((pos = strstr(message, "<fn ")) != nullptr)
    {
        // found <fn
        handleFn(pos, locos);
    }
    else if ((pos = strstr(message, "<sw ")) != nullptr)
    {
        // first go for "<sw"
        log4MC::debug("Received and ignored 'sw' command'.");
    }
    else if ((pos = strstr(message, "<clock ")) != nullptr)
    {
        // found clock
        log4MC::debug("Received and ignored 'clock' command'.");
    } // IGNORE THE REST
}

void MattzoBLEMQTTHandler::handleSys(const char *message, std::vector<BLELocomotive *> locos)
{
    char *cmd = nullptr;
    if (!XmlParser::tryReadCharAttr(message, "cmd", &cmd))
    {
        log4MC::warn("Received 'sys' command, but couldn't read 'cmd' attribute.");
        return;
    }

    if (strcmp(cmd, "ebreak") == 0 || strcmp(cmd, "stop") == 0 || strcmp(cmd, "shutdown") == 0)
    {
        log4MC::vlogf(LOG_INFO, "Received '%s' command. Stopping all locos.", cmd);

        // Upon receiving "stop", "ebreak" or "shutdown" system command from Rocrail, the global emergency break flag is set. Train will stop immediately.
        for (int i = 0; i < locos.size(); i++)
        {
            locos.at(i)->EmergencyBreak(true);
        }

        return;
    }

    if (strcmp(cmd, "go") == 0)
    {
        log4MC::info("Received 'go' command. Releasing e-break and starting all locos.");

        // Upon receiving "go" command, the emergency break flag is be released (i.e. pressing the light bulb in Rocview).
        for (int i = 0; i < locos.size(); i++)
        {
            locos.at(i)->EmergencyBreak(false);
        }

        return;
    }
}

void MattzoBLEMQTTHandler::handleLc(const char *message, std::vector<BLELocomotive *> locos)
{
    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr))
    {
        // Log error, ignore message.
        log4MC::warn("Received 'lc' command, but couldn't read 'addr' attribute.");
        return;
    }

    for (int i = 0; i < locos.size(); i++)
    {
        // Find loco by address.
        if (locos.at(i)->GetLocoAddress() == addr)
        {
            // Get target speed.
            int speed;
            if (!XmlParser::tryReadIntAttr(message, "V", &speed))
            {
                // Log error, ignore message.
                log4MC::warn("Received 'lc' command, but couldn't read 'V' attribute.");
                return;
            }

            // Get min speed.
            int minSpeed;
            if (!XmlParser::tryReadIntAttr(message, "V_min", &minSpeed))
            {
                // Log error, ignore message.
                log4MC::warn("Received 'lc' command, but couldn't read 'V_min' attribute.");
                return;
            }

            if (speed != 0 && speed < minSpeed)
            {
                // Requested speed is too low, we should ignore this command.
                log4MC::vlogf(LOG_DEBUG, "Received and ignored 'lc' command, because speed (%u) was below V_min (%u).", speed, minSpeed);
                return;
            }

            // Get max speed.
            int maxSpeed;
            if (!XmlParser::tryReadIntAttr(message, "V_max", &maxSpeed))
            {
                // Log error, ignore message.
                log4MC::warn("Received 'lc' command, but couldn't read 'V_max' attribute.");
                return;

            }

            // Get speed mode (percentage or km/h).
            char *mode;
            if (!XmlParser::tryReadCharAttr(message, "V_mode", &mode))
            {
                // Log error, ignore message.
                log4MC::warn("Received 'lc' command, but couldn't read 'V_mode' attribute.");
                return;

            }

            // Get direction (true=forward, false=backward).
            bool dirBool;
            if (!XmlParser::tryReadBoolAttr(message, "dir", &dirBool))
            {
                // Log error, ignore message.
                log4MC::warn("Received 'lc' command, but couldn't read 'dir' attribute.");
                return;
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

void MattzoBLEMQTTHandler::handleFn(const char *message, std::vector<BLELocomotive *> locos)
{
    // TODO: Ignore message.
    log4MC::warn("Received and ignored 'fn' command'.");
    return;

    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr))
    {
        // Log error, ignore message.
        log4MC::warn("Received 'fn' command' but couldn't read 'addr' attribute.");
        return;
    }

    for (int i = 0; i < locos.size(); i++)
    {
        // Find loco by address.
        if (locos.at(i)->GetLocoAddress() == addr)
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
                // Log error, ignore message.
                log4MC::warn("Received 'fn' command' but couldn't read 'fnchanged' attribute.");
                return;
            }

            // Query fnchangedstate attribute. This is the new state of the lights (true=on, false=off) of a channel.
            bool fnchangedstate;
            if (!XmlParser::tryReadBoolAttr(message, "fnchangedstate", &fnchangedstate))
            {
                // Log error, ignore message.
                log4MC::warn("Received 'fn' command' but couldn't read 'fnchangedstate' attribute.");
                return;
            }

            // Determine actual channel.
            // HubChannel channel = static_cast<HubChannel>(fnchanged);

            // TODO: Turn lights on for the requested channel (if channel has lights attached!).
            // locos[i]->SetLights(channel, fnchangedstate);
        }
    }
}