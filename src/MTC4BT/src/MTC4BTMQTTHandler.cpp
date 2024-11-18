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
/*
<lc id="V100" addr="1" prev_id="V100" shortid="" roadname="" owner="" color=""
number="" home="Shadow Station" desc="" dectype="" decfile="nmra-rp922.xml"
docu="" image="v100.png" imagenr="0" remark="" len="25" radius="0" weight="0"
nraxis="0" nrcars="0" manuid="" catnr="" purchased="" value="" identifier=""
show="true" active="true" useshortid="false" mint="0" throttlenr="0" manually="false"
bus="0" uid="0" secaddr="0" iid="" informall="false" oid="" prot="P" protver="1"
spcnt="255" secspcnt="255" fncnt="2" V_min="30" V_mid="50" V_cru="80" V_max="100" V_maxsec="14"
KMH_min="0" KMH_mid="0" KMH_cru="0" KMH_max="0" KMH_Rmin="0" KMH_Rmid="0" KMH_Rcru="0" KMH_Rmax="0"
KMH_Smin="0" KMH_Smid="0" KMH_Scru="0" KMH_Smax="0" V_Rmin="0" V_Rmid="0" V_Rcru="0" V_Rmax="0" V_Smin="0"
V_Smid="0" V_Scru="0" V_Smax="0" V_step="0" mass="0" minstep="0" maxstep="0" pwm="0" pwmcorrdiv="10"
Vmidpercent="30" Vmaxmin="20" Vmaxmax="255" Vmaxkmh="0" Vmidset="true" V_mode="kmh" invdir="false"
polarisation="true" regulated="true" restorefx="false" restorefxalways="false" restorespeed="false"
info4throttle="false" dirpause="0" adjustaccel="false" maxload="0" accelmin="0" accelmax="0" decelerate="0"
accelcv="3" accelcvindex="0" vmaxcv="5" vmaxcvindex="0" vmidcv="6" vmidcvindex="0" camhost="" camport="8081"
camtype="0" camfile="stream.mjpg" camskip="0" camoption="0" blockwaittime="10" maxwaittime="0" evttimer="0"
minenergypercentage="0" swaptimer="0" ent2incorr="100" priority="10" usescheduletime="false"
commuter="false" shortin="false" inatpre2in="false" usemanualroutes="false" useownwaittime="false"
startupscid="" startuptourid="" check2in="true" usedepartdelay="true" freeblockonenter="true"
reducespeedatenter="false" routespeedatenter="false" v0onswap="false" resetplacing="false"
manual="false" lookupschedule="false" lookupschedulevirtual="false" generated="false"
swapondir="false" screcord="false" decoupler="false" engine="diesel" cargo="none"
secondnextblock="false" secondnextblock4wait="false" era="0" class="" consist_syncfunmap="0"
standalone="false" consist_lightsoff="false" consist_syncfun="false" consist_synclights="false"
consist="" usebbt="false" bbtsteps="10" bbtstartinterval="10" bbtmaxdiff="250" bbtcorrection="25"
bbtkey="0" cvnrs="1,2,3,4,5,6,17,18,29" destblockid="" cmdDelay="0" pause="false" mode="stop"
fifotop="false" energypercentage="0" V="0" fx="0" throttleid="" trainlen="25" trainweight="0"
blockid="" blockenterid="sb01" resumeauto="false" modereason="" waittime="0" V_hint="cruise"
rdate="1730325551" runtime="37907" fn="false" blockenterside="true" cmd="modify" signalaspect=""
sid="0" dir="false" placing="true" modeevent="true" shunting="false" mtime="0" scidx="-1" scheduleid=""
tourid="" scheduleinithour="14" train="" V_realkmh="0" V_maxkmh="0" controlcode="" slavecode=""
server="infw075D54C4" gotoblockid="" homeside="0" wheeldiameter="13.000000" wheelbase="0"
maxincline="0" sernr="" coupler="" pwmkickstart="0" forcepriority="false" commuterblocks=""
commuterlevel="" waitallblocks="false" waitallblocksalt="false" departdelay="0" routestack="false"
stoponfailgoto="false" directgoto="false" engineFxType="" engineFxNr="0" sbt_decelerate="0"
sbt_interval="0" bat_accelerate="0" bat_interval="0" arrivetime="1730146471"/>

There could be fn's, but for the PU remote we are ignoring these.

This command is needed to get info about the locomotive afer it is seleced by the remote
*/
void MTC4BTMQTTHandler::handleInfoLc(const char *message)
{
    int addr = 0;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'addr' attribute.");
        return;
    }
    char *prev_id = nullptr;
    bool has_previd = XmlParser::tryReadCharAttr(message, "prev_id", &prev_id);
    if (prev_id)
        free(prev_id);
    if (!has_previd)
        return;
    // ignore all other lc commands on the info channel!

    // find the remote that controlls this locomotive, this part is also
    // in the regular lc command, this is only to get the initial values
    // of the locomotive
    std::vector<lc *> remotes = controller->findRemoteByAddr(addr);
    if (remotes.size() != 0) {
        lc *currentLC = getCurrentLcSpeed(message, has_previd, remotes[0]->invdir);
        // copy all found values to the remote(s)
        for (int i = 0; i < remotes.size(); i++) {
            remotes[i]->vModePercent = currentLC->vModePercent;
            remotes[i]->V = currentLC->V;
            remotes[i]->Vmax = currentLC->Vmax;
            remotes[i]->initiated = true;
            remotes[i]->invdir = currentLC->invdir;
        }
        delete( currentLC);
        remotes.clear(); // DO NOT FREE, THEY ARE REFERENCED BY THE REMOTES!
    }
}

lc *MTC4BTMQTTHandler::getCurrentLcSpeed(const char *message, bool has_previd, bool invdir)
{
    // there are remotes which handle this locomotive
    char *Vmode;
    lc *currentLc = new lc();
    XmlParser::tryReadIntAttr(message, "V_max", &(currentLc->Vmax));
    // XMLParser::tryReadIntAttr(message, "V_Rmax", &(currentLoc->VRmax));
    // XMLParser::tryReadIntAttr(message, "V_Smax", &(currentLoc->VSmax));
    if (XmlParser::tryReadCharAttr(message, "V_mode", &Vmode)) {
        if (strstr(Vmode, "kmh") != nullptr) {
            currentLc->vModePercent = false;
        } else {
            currentLc->vModePercent = true;
        }
        free(Vmode);
    }
    XmlParser::tryReadIntAttr(message, "V", &(currentLc->V));      // current speed
    XmlParser::tryReadBoolAttr(message, "dir", &(currentLc->dir)); // current direction
    // placing="true" blockenterside="true"
    // are we in the info message?
    if (has_previd) {
        bool placing, blockenterside;
        XmlParser::tryReadBoolAttr(message, "placing", &(placing));
        // XmlParser::tryReadBoolAttr(message, "blockenterside", &(blockenterside));
        currentLc->invdir = !placing; // ignore stuff
    } else {
        currentLc->invdir = invdir;
    }

    if (!currentLc->dir) {
        currentLc->V = -currentLc->V;
        currentLc->dir = true;
    }

    if (currentLc->invdir) {
        currentLc->V = -currentLc->V;
    }
    if (currentLc->newSpeed != currentLc->V) {
        // only change if we have a speed change, ignore 0 because that is our
        currentLc->newSpeed = currentLc->V;
    }
    return currentLc;
}

void MTC4BTMQTTHandler::handleLc(const char *message)
{
    int addr;
    if (!XmlParser::tryReadIntAttr(message, "addr", &addr)) {
        // Log error, ignore message.
        log4MC::warn("MQTT: Received 'lc' command, but couldn't read 'addr' attribute.");
        return;
    }
    // find the remote that controlls this locomotive, this part is also
    // in the regular lc command, this is only to get the initial values
    // of the locomotive
    std::vector<lc *> remotes = controller->findRemoteByAddr(addr);
    if (remotes.size() != 0) {
        lc *currentLC = getCurrentLcSpeed(message, false, remotes[0]->invdir);
        // copy all found values to the remote(s)
        for (int i = 0; i < remotes.size(); i++) {
            remotes[i]->vModePercent = currentLC->vModePercent;
            remotes[i]->V = currentLC->V;
            remotes[i]->Vmax = currentLC->Vmax;
            remotes[i]->initiated = true;
            remotes[i]->invdir = currentLC->invdir;
        }
        delete( currentLC );
        remotes.clear(); // DO NOT FREE, THEY ARE REFECED BY THE REMOTES!
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

void MTC4BTMQTTHandler::pubGetShortLcList()
{
    mqttSubscriberClient.publish("rocrail/service/client", "<model cmd=\"lclist\" val=\"short\"/>");
}

void MTC4BTMQTTHandler::pubGetLcInfo(char *locid)
{
    // get current info of the loc from rocrail and start following it!
    char request[200];
    snprintf(request, 200, "<model cmd=\"lcprops\" val=\"%s\"/>", locid);
    mqttSubscriberClient.publish("rocrail/service/client", request);
}

void MTC4BTMQTTHandler::pubLcSpeed(char *locid, int addr, long locV)
{
    char request[200];
    bool dir = locV > 0;
    snprintf(request, 200, "<lc id=\"%s\" addr=\"%d\" dir=\"%s\" V=\"%ld\"/>", locid, addr,
             dir ? "true" : "false", abs(locV));
    mqttSubscriberClient.publish("rocrail/service/client", request);
}
