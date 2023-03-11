#pragma once

#include <map>

enum BLEHubChannel {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    OnboardLED = 99
};

// String switch paridgam
struct bleHubChannelMap : public std::map<std::string, BLEHubChannel> {
    bleHubChannelMap()
    {
        this->operator[]("A") = BLEHubChannel::A;
        this->operator[]("a") = BLEHubChannel::A;
        this->operator[]("B") = BLEHubChannel::B;
        this->operator[]("b") = BLEHubChannel::B;
        this->operator[]("C") = BLEHubChannel::C;
        this->operator[]("c") = BLEHubChannel::C;
        this->operator[]("D") = BLEHubChannel::D;
        this->operator[]("d") = BLEHubChannel::D;
        this->operator[]("LED") = BLEHubChannel::OnboardLED;
        this->operator[]("Led") = BLEHubChannel::OnboardLED;
        this->operator[]("led") = BLEHubChannel::OnboardLED;
    };
    ~bleHubChannelMap() {}
};