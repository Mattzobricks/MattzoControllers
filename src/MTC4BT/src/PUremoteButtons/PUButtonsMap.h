#pragma once

#include <map>

typedef enum {
    Bnone = 0,
    Aplus,
    Ared,
    Amin,
    Bplus,
    Bred,
    Bmin,
    Green,
    maxButton // this should alway be the last item
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
        this->operator[]("green") = PUbutton::Green;
    };
    ~PUbuttonMap() {}
};
