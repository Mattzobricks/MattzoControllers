#pragma once

#include <map>

enum ChannelType {
    EspPinChannel = 0,
    BleHubChannel
};

// String switch paridgam
struct channelTypeMap : public std::map<std::string, ChannelType> {
    channelTypeMap()
    {
        this->operator[]("espPin") = ChannelType::EspPinChannel;
        this->operator[]("bleHub") = ChannelType::BleHubChannel;
    };
    ~channelTypeMap() {}
};

enum DeviceType {
    Nothing,
    Motor,
    Light,
    StatusLight
};

// String switch paridgam
struct deviceTypeMap : public std::map<std::string, DeviceType> {
    deviceTypeMap()
    {
        this->operator[]("") = DeviceType::Nothing;
        this->operator[]("nothing") = DeviceType::Nothing;
        this->operator[]("motor") = DeviceType::Motor;
        this->operator[]("light") = DeviceType::Light;
        this->operator[]("status") = DeviceType::StatusLight;
    };
    ~deviceTypeMap() {}
};

enum HubLedColor {
    BLACK = 0,
    PINK = 1,
    PURPLE = 2,
    BLUE = 3,
    LIGHTBLUE = 4,
    CYAN = 5,
    GREEN = 6,
    YELLOW = 7,
    ORANGE = 8,
    RED = 9,
    WHITE = 10,
    NUM_COLORS,
    NONE = 255
};

// String switch paridgam
struct hubLedColorMap : public std::map<std::string, HubLedColor> {
    hubLedColorMap()
    {
        this->operator[]("") = HubLedColor::NONE;
        this->operator[]("off") = HubLedColor::BLACK;
        this->operator[]("black") = HubLedColor::BLACK;
        this->operator[]("pink") = HubLedColor::PINK;
        this->operator[]("purple") = HubLedColor::PURPLE;
        this->operator[]("blue") = HubLedColor::BLUE;
        this->operator[]("lightblue") = HubLedColor::LIGHTBLUE;
        this->operator[]("cyan") = HubLedColor::CYAN;
        this->operator[]("green") = HubLedColor::GREEN;
        this->operator[]("yellow") = HubLedColor::YELLOW;
        this->operator[]("orange") = HubLedColor::ORANGE;
        this->operator[]("red") = HubLedColor::RED;
        this->operator[]("white") = HubLedColor::WHITE;
    };
    ~hubLedColorMap() {}
};

enum MCTriggerSource {
    // Locomotive.
    Loco,

    // RocRail.
    RocRail
};

// String switch paridgam
struct triggerSourceMap : public std::map<std::string, MCTriggerSource> {
    triggerSourceMap()
    {
        this->operator[]("") = MCTriggerSource::Loco;
        this->operator[]("loco") = MCTriggerSource::Loco;
        this->operator[]("rr") = MCTriggerSource::RocRail;
    };
    ~triggerSourceMap() {}
};

// Function supported by the generic controller.
enum MCFunction {
    Status,
    F0,
    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,
    F13,
    F14,
    F15,
    F16,
    F17,
    F18,
    F19,
    F20,
    F21,
    F22,
    F23,
    F24,
    F25,
    F26,
    F27,
    F28,
    F29,
    F30,
    F31,
    F32
};

// String switch paridgam
struct functionMap : public std::map<std::string, MCFunction> {
    functionMap()
    {
        this->operator[]("status") = MCFunction::Status;
        this->operator[]("f0") = MCFunction::F0;
        this->operator[]("f1") = MCFunction::F1;
        this->operator[]("f2") = MCFunction::F2;
        this->operator[]("f3") = MCFunction::F3;
        this->operator[]("f4") = MCFunction::F4;
        this->operator[]("f5") = MCFunction::F5;
        this->operator[]("f6") = MCFunction::F6;
        this->operator[]("f7") = MCFunction::F7;
        this->operator[]("f8") = MCFunction::F8;
        this->operator[]("f9") = MCFunction::F9;
        this->operator[]("f10") = MCFunction::F10;
        this->operator[]("f11") = MCFunction::F11;
        this->operator[]("f12") = MCFunction::F12;
        this->operator[]("f13") = MCFunction::F13;
        this->operator[]("f14") = MCFunction::F14;
        this->operator[]("f15") = MCFunction::F15;
        this->operator[]("f16") = MCFunction::F16;
        this->operator[]("f17") = MCFunction::F17;
        this->operator[]("f18") = MCFunction::F18;
        this->operator[]("f19") = MCFunction::F19;
        this->operator[]("f20") = MCFunction::F20;
        this->operator[]("f21") = MCFunction::F21;
        this->operator[]("f22") = MCFunction::F22;
        this->operator[]("f23") = MCFunction::F23;
        this->operator[]("f24") = MCFunction::F24;
        this->operator[]("f25") = MCFunction::F25;
        this->operator[]("f26") = MCFunction::F26;
        this->operator[]("f27") = MCFunction::F27;
        this->operator[]("f28") = MCFunction::F28;
        this->operator[]("f29") = MCFunction::F29;
        this->operator[]("f30") = MCFunction::F30;
        this->operator[]("f31") = MCFunction::F31;
        this->operator[]("f32") = MCFunction::F32;
    };
    ~functionMap() {}
};