#pragma once

#include "MTC4BTConfiguration.h"

class MTC4BTController
{
public:
    MTC4BTController(MTC4BTConfiguration *config);

    DeviceConfiguration *GetStatusLed();

private:
    MTC4BTConfiguration *_config;
};