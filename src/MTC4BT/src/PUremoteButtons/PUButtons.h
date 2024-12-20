#pragma once

#include <map>
#include <vector>

#include "rocrailitems/RRtypes.h"

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
        this->operator[]("gree") = RRaction::RRgreen;
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

typedef enum {
    Bnone = 0,
    Aplus,
    Ared,
    Amin,
    Bplus,
    Bred,
    Bmin,
    Green,
    maxButton = Green // this should alway be the last item
} PUbutton;

struct PUbuttonMap : public std::map<std::string, PUbutton> {
    PUbuttonMap()
    {
        this->operator[]("A+") = PUbutton::Aplus;
        this->operator[]("Ared") = PUbutton::Ared;
        this->operator[]("A-") = PUbutton::Amin;
        this->operator[]("B+") = PUbutton::Bplus;
        this->operator[]("Bred") = PUbutton::Bred;
        this->operator[]("B-") = PUbutton::Bmin;
        this->operator[]("Green") = PUbutton::Green;
    };
    ~PUbuttonMap() {}
};

class PUbuttonByType
{
  public:
    PUbuttonByType();
    ~PUbuttonByType();

    void setButton(RRdevice device, PUbutton button,RRaction action);
    RRaction getButton(RRdevice device, PUbutton button);

  protected:
    RRaction buttonByType[RRmaxdevice][maxButton];
};

class PUbuttonByAction
{
};