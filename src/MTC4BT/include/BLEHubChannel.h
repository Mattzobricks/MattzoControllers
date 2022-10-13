#pragma once

#include <map>

enum BLEHubChannel {
    A,
    B,
    C,
    D
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
    };
    ~bleHubChannelMap() {}
};