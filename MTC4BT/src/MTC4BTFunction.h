#pragma once

#include <map>

// Function supported by this controller.
enum MTC4BTFunction
{
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
struct functionMap : public std::map<std::string, MTC4BTFunction>
{
    functionMap()
    {
        this->operator[]("status") = MTC4BTFunction::Status;
        this->operator[]("f0") = MTC4BTFunction::F0;
        this->operator[]("f1") = MTC4BTFunction::F1;
        this->operator[]("f2") = MTC4BTFunction::F2;
        this->operator[]("f3") = MTC4BTFunction::F3;
        this->operator[]("f4") = MTC4BTFunction::F4;
        this->operator[]("f5") = MTC4BTFunction::F5;
        this->operator[]("f6") = MTC4BTFunction::F6;
        this->operator[]("f7") = MTC4BTFunction::F7;
        this->operator[]("f8") = MTC4BTFunction::F8;
        this->operator[]("f9") = MTC4BTFunction::F9;
        this->operator[]("f10") = MTC4BTFunction::F10;
        this->operator[]("f11") = MTC4BTFunction::F11;
        this->operator[]("f12") = MTC4BTFunction::F12;
        this->operator[]("f13") = MTC4BTFunction::F13;
        this->operator[]("f14") = MTC4BTFunction::F14;
        this->operator[]("f15") = MTC4BTFunction::F15;
        this->operator[]("f16") = MTC4BTFunction::F16;
        this->operator[]("f17") = MTC4BTFunction::F17;
        this->operator[]("f18") = MTC4BTFunction::F18;
        this->operator[]("f19") = MTC4BTFunction::F19;
        this->operator[]("f20") = MTC4BTFunction::F20;
        this->operator[]("f21") = MTC4BTFunction::F21;
        this->operator[]("f22") = MTC4BTFunction::F22;
        this->operator[]("f23") = MTC4BTFunction::F23;
        this->operator[]("f24") = MTC4BTFunction::F24;
        this->operator[]("f25") = MTC4BTFunction::F25;
        this->operator[]("f26") = MTC4BTFunction::F26;
        this->operator[]("f27") = MTC4BTFunction::F27;
        this->operator[]("f28") = MTC4BTFunction::F28;
        this->operator[]("f29") = MTC4BTFunction::F29;
        this->operator[]("f30") = MTC4BTFunction::F30;
        this->operator[]("f31") = MTC4BTFunction::F31;
        this->operator[]("f32") = MTC4BTFunction::F32;
    };
    ~functionMap() {}
};
