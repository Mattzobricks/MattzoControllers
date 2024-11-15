#include "MTC4BTMQTTHandler.h"
#include "MTC4BTController.h"
#include "log4MC.h"

#include "rocrailitems/lclist.h"

// it is globaly devined in main.cpp
extern MTC4BTController *controller;

void MTC4BTMQTTHandler::Handle(const char *message)
{
    char *pos;

    // parse the rocrail mqtt messages, all of them,
    if ((pos = strstr(message, "<sys ")) != nullptr) {
        // found <sys
        handleSys(pos);
    } else if ((pos = strstr(message, "<lc ")) != nullptr) {
        // found <lc
        handleLc(pos);
    } else if ((pos = strstr(message, "<fn ")) != nullptr) {
        // found <fn
        handleFn(pos);
    } // IGNORE THE REST
}

void MTC4BTMQTTHandler::infoHandle(const char *message)
{
    char *pos;
    if ((pos = strstr(message, "<lclist")) != nullptr) {
        handleLCList(pos + 5);
    } // IGNORE THE REST
}

void MTC4BTMQTTHandler::handleSys(const char *message)
{
    char *cmd = nullptr;
    if (!XmlParser::tryReadCharAttr(message, "cmd", &cmd)) {
        log4MC::warn("MQTT: Received 'sys' command, but couldn't read 'cmd' attribute.");
        if (cmd)
            free(cmd);
        return;
    }

    if (strcmp(cmd, "ebreak") == 0 || strcmp(cmd, "stop") == 0 || strcmp(cmd, "shutdown") == 0) {
        log4MC::vlogf(LOG_INFO, "MQTT: Received '%s' command. Stopping all locos.", cmd);

        // Upon receiving "stop", "ebreak" or "shutdown" system command from Rocrail, the global emergency brake flag is set. All trains will stop immediately.
        controller->HandleSys(true);
        if (cmd)
            free(cmd);
        return;
    }

    if (strcmp(cmd, "go") == 0) {
        log4MC::info("MQTT: Received 'go' command. Releasing e-brake and resuming all locos.");

        // Upon receiving "go" command, the emergency brake flag is released (i.e. pressing the light bulb in Rocview).
        controller->HandleSys(false);
        if (cmd)
            free(cmd);
        return;
    }

    // free if not any of the above commands!
    if (cmd)
        free(cmd);
}

void MTC4BTMQTTHandler::handleLc(const char *message)
{
    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'addr' attribute.");
        return;
    }

    if (!controller->HasLocomotive(addr)) {
        // Not a loco under our control. Ignore message.
        // log4MC::vlogf(LOG_DEBUG, "MQTT: Loco with address '%u' is not under our control. Lc command ignored.", addr);
        return;
    }

    // Get target speed.
    int speed;
    if (!XmlParser::tryReadIntAttr(message, "V", &speed)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'V' attribute.");
        return;
    }

    // Get min speed.
    int minSpeed;
    if (!XmlParser::tryReadIntAttr(message, "V_min", &minSpeed)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'V_min' attribute.");
        return;
    }

    if (speed != 0 && speed < minSpeed) {
        // Requested speed is too low, we should ignore this command.
        log4MC::vlogf(LOG_DEBUG, "MQTT: Received and ignored 'lc' command, because speed (%u) was below V_min (%u).", speed, minSpeed);
        return;
    }

    // Get max speed.
    int maxSpeed;
    if (!XmlParser::tryReadIntAttr(message, "V_max", &maxSpeed)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'V_max' attribute.");
        return;
    }

    // Get speed mode (percentage or km/h).
    char *mode = nullptr;
    if (!XmlParser::tryReadCharAttr(message, "V_mode", &mode)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'V_mode' attribute.");
        if (mode)
            free(mode);
        return;
    }

    // Get direction (true=forward, false=backward).
    bool dirForward;
    if (!XmlParser::tryReadBoolAttr(message, "dir", &dirForward)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'dir' attribute.");
        if (mode)
            free(mode);
        return;
    }

    // Ask controller to handle the loco command.
    controller->HandleLc(addr, speed, minSpeed, maxSpeed, mode, dirForward);
    if (mode)
        free(mode);
}

// Compares two intervals according to starting times.
bool compareAddress(lc *i1, lc *i2)
{
    return (i1->addr < i2->addr);
}

void MTC4BTMQTTHandler::handleLCList(const char *message)
{
    const char *pos = message;
    log4MC::vlogf(LOG_DEBUG, "Going for delete loco list.");
    for (auto p : locs) {
        delete p;
    }
    locs.clear(); // delete all previous locs
    log4MC::vlogf(LOG_DEBUG, "Going for loco list.");
    lc *loc = NULL;
    while ((pos = strstr(pos, "<lc")) != nullptr) {
        // found a loc
        char *id = NULL;
        int addr = 0;
        if (!XmlParser::tryReadCharAttr(pos, "id", &id)) {
            // id can not be empty
        }
        if (!XmlParser::tryReadIntAttr(pos, "addr", &addr)) {
            // just ignore for now
        }
        loc = new lc(id, addr, NULL, 0, 0, 0);
        locs.push_back(loc);
        pos++;
    }
    // sort locs by address
    sort(locs.begin(), locs.end(), compareAddress);
    // find the range index offeset

    controller->handleLCList();
    log4MC::vlogf(LOG_DEBUG, "Got for loco list.");
}

void MTC4BTMQTTHandler::handleFn(const char *message)
{
    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr)) {
        // Log error, ignore message.
        // log4MC::warn("MQTT: Received 'fn' command' but couldn't read 'addr' attribute.");
        return;
    }

    if (!controller->HasLocomotive(addr)) {
        // Not a loco under our control. Stop parsing and ignore message.
        // log4MC::vlogf(LOG_DEBUG, "MQTT: Loco with address '%u' is not under our control. Lc command ignored.", addr);
        return;
    }

    // Get number of function that changed.
    int fnchanged;
    if (!XmlParser::tryReadIntAttr(message, "fnchanged", &fnchanged)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'fn' command' but couldn't read 'fnchanged' attribute.");
        return;
    }

    // Query fnchangedstate attribute. This is the new state of the function (true=on, false=off).
    bool fnchangedstate;
    if (!XmlParser::tryReadBoolAttr(message, fnchanged == 0 ? "fn" : "fnchangedstate", &fnchangedstate)) {
        // Log error, ignore message.
        log4MC::vlogf(LOG_WARNING, "MQTT: Received 'fn' command' for 'f%u' but couldn't read '%s' attribute.", fnchanged, fnchanged == 0 ? "fn" : "fnchangedstate");
        return;
    }

    // Convert function number to string (format: fX);
    static char fnId[3];
    sprintf(fnId, "f%u", fnchanged);

    // Ask controller to handle the function.
    controller->HandleTrigger(addr, MCTriggerSource::RocRail, "fnchanged", fnId, fnchangedstate ? "on" : "off");
}