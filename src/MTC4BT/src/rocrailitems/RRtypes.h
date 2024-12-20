#pragma once

/* This files contains all the rocrail types and maps to it
 */

#include <map>

typedef enum {
    RRloco = 0,
    RRswitch,
    RRsignal,
    RRoutput,
    RRmaxdevice = RRoutput
} RRdevice;

struct RRdeviceMap : public std::map<std::string, RRdevice> {
    RRdeviceMap()
    {
        this->operator[]("loco") = RRdevice::RRloco;
        this->operator[]("switch") = RRdevice::RRswitch;
        this->operator[]("signal") = RRdevice::RRsignal;
        this->operator[]("output") = RRdevice::RRoutput;
    };
    ~RRdeviceMap() {}
};
