#pragma once

/* This files contains all the rocrail types and maps to it
 */

#include <map>

typedef enum {
    RRloco = 1,
    RRswitch,
    RRtripleswitch,
    RRsignal,
    RRoutput,
    RRmaxdevice // This should be the last item
} RRdevice;

struct RRdeviceMap : public std::map<std::string, RRdevice> {
    RRdeviceMap()
    {
        this->operator[]("loco") = RRdevice::RRloco;
        this->operator[]("switch") = RRdevice::RRswitch;
        this->operator[]("switch3") = RRdevice::RRtripleswitch;
        this->operator[]("signal") = RRdevice::RRsignal;
        this->operator[]("output") = RRdevice::RRoutput;
    };
    ~RRdeviceMap() {}
};

typedef enum {
    RRnoop = 0,
    RRinc,
    RRdec,
    RRstop,
    RRflip,
    RRon,
    RRoff,
    RRgreen,
    RRred,
    RRyellow,
    RRwhite,
    RRleft,
    RRright,
    RRstraight,
    RRturnout,
    RRgo,
    RRebrake, // everything after this is reserved
    navUp,
    navDown
} RRaction;

struct RRactionMap : public std::map<std::string, RRaction> {
    RRactionMap()
    {
        this->operator[]("noop") = RRaction::RRnoop;
        this->operator[]("inc") = RRaction::RRinc;
        this->operator[]("dec") = RRaction::RRdec;
        this->operator[]("stop") = RRaction::RRstop;
        this->operator[]("flip") = RRaction::RRflip;
        this->operator[]("on") = RRaction::RRon;
        this->operator[]("off") = RRaction::RRoff;
        this->operator[]("green") = RRaction::RRgreen;
        this->operator[]("red") = RRaction::RRred;
        this->operator[]("yellow") = RRaction::RRyellow;
        this->operator[]("white") = RRaction::RRwhite;
        this->operator[]("left") = RRaction::RRleft;
        this->operator[]("right") = RRaction::RRright;
        this->operator[]("straight") = RRaction::RRstraight;
        this->operator[]("turnout") = RRaction::RRturnout;
        this->operator[]("go") = RRaction::RRgo;
        this->operator[]("ebrake") = RRaction::RRebrake;
    };
    ~RRactionMap() {}
};
