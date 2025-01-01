/*
 * MTC4BTMQTTHandler
 * Handle all mqtt.
 */

#include "MTC4BTMQTTHandler.h"
#include "MTC4BTController.h"
#include "log4MC.h"

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
    } else if ((pos = strstr(message, "<lc ")) != nullptr) {
        // found <lc
        handleInfoLc(pos + 2);
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

void MTC4BTMQTTHandler::handleInfoLc(const char *message)
{
    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr)) {
        // Log error, ignore message.
        log4MC::warn("Received MQTT 'lc' command with missing addr attribute. Message skipped.");
        return;
    }

    std::vector<lc *> remotes = controller->findRemoteByAddr(addr);
    if (remotes.size() != 0) {
        // log4MC::vlogf(LOG_DEBUG, "%s: found remote for loco addr %d.",__func__, addr);
        // copy loco values to the remote(s)
        for (int i = 0; i < remotes.size(); i++) {
            int v;
            if (XmlParser::tryReadIntAttr(message, "V", &v)) {
                if (remotes[i]->V != v) {
                    remotes[i]->V = v;
                    log4MC::vlogf(LOG_DEBUG, "%s: Loco %d updated: v %d",__func__, addr, v);
                }
            } else {
                // log4MC::vlogf(LOG_DEBUG, "%s: Loco %d: missing v attribute. Attribute skipped.",__func__, addr);
            }

            char *vMode;
            if (XmlParser::tryReadCharAttr(message, "V_mode", &vMode)) {
                bool vModePercent = strstr(vMode, "percent") != nullptr;
                if (remotes[i]->vModePercent != vModePercent) {
                    remotes[i]->vModePercent = vModePercent;
                    log4MC::vlogf(LOG_DEBUG, "%s: Loco %d updated: vmode %s",__func__, addr, vModePercent ? "percent" : "absolute");
                }
                free(vMode);
            } else {
                // log4MC::vlogf(LOG_DEBUG, "%s: Loco %d: missing V_mode attribute. Attribute skipped.",__func__, addr);
            }

            bool dir;
            if (XmlParser::tryReadBoolAttr(message, "dir", &dir)) {
                if (remotes[i]->dir != dir) {
                    remotes[i]->dir = dir;
                    log4MC::vlogf(LOG_DEBUG, "%s: Loco %d updated: dir %s",__func__, addr, dir ? "forward" : "backwards");
                }
            } else {
                // log4MC::vlogf(LOG_DEBUG, "%s: Loco %d: missing dir attribute. Attribute skipped.",__func__, addr);
            }

            bool placing;
            if (XmlParser::tryReadBoolAttr(message, "placing", &placing)) {
                if (remotes[i]->placing != placing) {
                    remotes[i]->placing = placing;
                    log4MC::vlogf(LOG_DEBUG, "%s: Loco %d updated: placing %s",__func__, addr, placing ? "forward" : "reverse");
                }
            } else {
                // log4MC::vlogf(LOG_DEBUG, "%s: Loco %d: missing placing attribute. Attribute skipped.",__func__, addr);
            }

            int vMax;
            if (XmlParser::tryReadIntAttr(message, "V_max", &vMax)) {
                if (remotes[i]->Vmax != vMax) {
                    remotes[i]->Vmax = vMax;
                    log4MC::vlogf(LOG_DEBUG, "%s: Loco %d updated: vmax %d",__func__, addr, vMax);
                }
            } else {
                // log4MC::vlogf(LOG_DEBUG, "%s: Loco %d: missing V_max attribute. Attribute skipped.",__func__, addr);
            }

            remotes[i]->initiated = true;

            // log4MC::vlogf(LOG_DEBUG, "%s: Loco %d status: speed %d, vmax %d, vmode %s, dir %d, placing %s",__func__, addr, remotes[i]->V, remotes[i]->Vmax, remotes[i]->vModePercent ? "1" : "0", remotes[i]->dir, remotes[i]->placing ? "1" : "0");
        }
        remotes.clear(); // DO NOT FREE, THEY ARE REFERENCED BY THE REMOTES!
    }
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

void MTC4BTMQTTHandler::pubEBrake()
{
    mqttSubscriberClient.publish(MQTT_CLIENTTOPIC, "<sys cmd=\"ebreak\" informall=\"true\"/>");
}
void MTC4BTMQTTHandler::pubGo()
{
    mqttSubscriberClient.publish(MQTT_CLIENTTOPIC, "<sys cmd=\"go\" informall=\"true\"/>");
}

void MTC4BTMQTTHandler::pubGetShortLcList()
{
    mqttSubscriberClient.publish(MQTT_CLIENTTOPIC, "<model cmd=\"lclist\" val=\"short\"/>");
}

void MTC4BTMQTTHandler::pubGetLcInfo(char *locid)
{
    // get current info of the loc from rocrail and start following it!
    char request[200];
    snprintf(request, 200, "<model cmd=\"lcprops\" val=\"%s\"/>", locid);
    mqttSubscriberClient.publish(MQTT_CLIENTTOPIC, request);
}

void MTC4BTMQTTHandler::pubLcSpeed(char *locid, long locV)
{
    char request[200];
    bool dir = locV >= 0;
    snprintf(request, 200, "<lc id=\"%s\" dir=\"%s\" V=\"%ld\"/>", locid, dir ? "true" : "false", abs(locV));
    mqttSubscriberClient.publish(MQTT_CLIENTTOPIC, request);
}

void MTC4BTMQTTHandler::pubFlip(RRdevice device, char *id)
{
    char request[200];
    bool knownDevice = true;
    switch (device) {
    case RRsignal:
        snprintf(request, 200, "<sg id=\"%s\"  cmd=\"flip\" manualcmd=\"true\"/>", id);
        break;
    case RRswitch:
        snprintf(request, 200, "<sw id=\"%s\"  cmd=\"flip\" manualcmd=\"true\"/>", id);
        break;
    case RRoutput:
        snprintf(request, 200, "<co id=\"%s\"  cmd=\"flip\" manualcmd=\"true\"/>", id);
        break;
    default:
        knownDevice = false;
        break;
    }
    if (knownDevice)
        mqttSubscriberClient.publish(MQTT_CLIENTTOPIC, request);
}

void MTC4BTMQTTHandler::pubCo(RRaction action, char *id)
{
    char request[200];
    snprintf(request, 200, "<co id=\"%s\"  cmd=\"%s\"/>", id, action == RRon ? "on" : "off");
    mqttSubscriberClient.publish("rocrail/service/client", request);
}

void MTC4BTMQTTHandler::pubSg(RRaction action, char *id)
{
    char request[200];
    snprintf(request, 200, "<sg id=\"%s\"  cmd=\"%s\"/>", id, action == RRgreen ? "green" : (action == RRred ? "red" : (action == RRyellow ? "yellow" : "white")));
    mqttSubscriberClient.publish("rocrail/service/client", request);
}

void MTC4BTMQTTHandler::pubSw(RRaction action, char *id)
{
    char request[200];
    snprintf(request, 200, "<sw id=\"%s\"  cmd=\"%s\"/>", id, action == RRleft ? "left" : (action == RRright ? "right" : (action == RRstraight ? "straight" : "turnout")));
    mqttSubscriberClient.publish("rocrail/service/client", request);
}