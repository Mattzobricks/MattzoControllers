#pragma once

enum HardwareType
{
    EspPin = 0,
    BleHub
};

// String switch paridgam
struct hardwareTypeMap : public std::map<std::string, HardwareType>
{
    hardwareTypeMap()
    {
        this->operator[]("espPin") = HardwareType::EspPin;
        this->operator[]("bleHub") = HardwareType::BleHub;
    };
    ~hardwareTypeMap() {}
};