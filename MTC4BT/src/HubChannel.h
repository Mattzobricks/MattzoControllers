#pragma once

#include <map>

enum HubChannel
{
  A,
  B,
  C,
  D
};

// String switch paridgam   
struct hubChannelMap : public std::map<std::string, HubChannel>
{
    hubChannelMap()
    {
        this->operator[]("A") = HubChannel::A;
        this->operator[]("a") = HubChannel::A;
        this->operator[]("B") = HubChannel::B;
        this->operator[]("b") = HubChannel::B;
        this->operator[]("C") = HubChannel::C;
        this->operator[]("c") = HubChannel::C;
        this->operator[]("D") = HubChannel::D;
        this->operator[]("d") = HubChannel::D;
    };
    ~hubChannelMap(){}
};

enum HubChannelDirection
{
  FORWARD,
  REVERSE
};

// String switch paridgam   
struct hubChannelDirectionMap : public std::map<std::string, HubChannelDirection>
{
    hubChannelDirectionMap()
    {
        this->operator[]("forward") = HubChannelDirection::FORWARD;
        this->operator[]("reverse") = HubChannelDirection::REVERSE;
    };
    ~hubChannelDirectionMap(){}
};
