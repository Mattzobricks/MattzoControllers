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
    RRfn_noop = 0, // default if no fn-action is configured
    RRfn_on,
    RRfn_off,
    RRfn_push,
    RRfn_flip
} RRfnAction;

struct RRfnActionMap : public std::map<std::string, RRfnAction> {
    RRfnActionMap()
    {
        this->operator[]("on") = RRfnAction::RRfn_on;
        this->operator[]("off") = RRfnAction::RRfn_off;
        this->operator[]("push") = RRfnAction::RRfn_push;
        this->operator[]("flip") = RRfnAction::RRfn_flip;
    };
    ~RRfnActionMap() {}
};

typedef enum {
    RRnoop = 0,
    RRfn0,
    RRfn1,
    RRfn2,
    RRfn3,
    RRfn4,
    RRfn5,
    RRfn6,
    RRfn7,
    RRfn8,
    RRfn9,
    RRfn10,
    RRfn11,
    RRfn12,
    RRfn13,
    RRfn14,
    RRfn15,
    RRfn16,
    RRfn17,
    RRfn18,
    RRfn19,
    RRfn20,
    RRfn21,
    RRfn22,
    RRfn23,
    RRfn24,
    RRfn25,
    RRfn26,
    RRfn27,
    RRfn28,
    RRfn29,
    RRfn30,
    RRfn31,
    RRfn32,
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
        this->operator[]("fn0") = RRaction::RRfn0;
        this->operator[]("fn1") = RRaction::RRfn1;
        this->operator[]("fn2") = RRaction::RRfn2;
        this->operator[]("fn3") = RRaction::RRfn3;
        this->operator[]("fn4") = RRaction::RRfn4;
        this->operator[]("fn5") = RRaction::RRfn5;
        this->operator[]("fn6") = RRaction::RRfn6;
        this->operator[]("fn7") = RRaction::RRfn7;
        this->operator[]("fn8") = RRaction::RRfn8;
        this->operator[]("fn9") = RRaction::RRfn9;
        this->operator[]("fn10") = RRaction::RRfn10;
        this->operator[]("fn11") = RRaction::RRfn11;
        this->operator[]("fn12") = RRaction::RRfn12;
        this->operator[]("fn13") = RRaction::RRfn13;
        this->operator[]("fn14") = RRaction::RRfn14;
        this->operator[]("fn15") = RRaction::RRfn15;
        this->operator[]("fn16") = RRaction::RRfn16;
        this->operator[]("fn17") = RRaction::RRfn17;
        this->operator[]("fn18") = RRaction::RRfn18;
        this->operator[]("fn19") = RRaction::RRfn19;
        this->operator[]("fn20") = RRaction::RRfn20;
        this->operator[]("fn21") = RRaction::RRfn21;
        this->operator[]("fn22") = RRaction::RRfn22;
        this->operator[]("fn23") = RRaction::RRfn23;
        this->operator[]("fn24") = RRaction::RRfn24;
        this->operator[]("fn25") = RRaction::RRfn25;
        this->operator[]("fn26") = RRaction::RRfn26;
        this->operator[]("fn27") = RRaction::RRfn27;
        this->operator[]("fn28") = RRaction::RRfn28;
        this->operator[]("fn29") = RRaction::RRfn29;
        this->operator[]("fn30") = RRaction::RRfn30;
        this->operator[]("fn31") = RRaction::RRfn31;
        this->operator[]("fn32") = RRaction::RRfn32;
    };
    ~RRactionMap() {}
};
