#pragma once

#include "MCLoggingSerialConfiguration.h"
#include "MCLoggingSyslogConfiguration.h"

struct MCLoggingConfiguration {
  public:
    std::string MinLevel;
    MCLoggingSerialConfiguration *Serial;
    MCLoggingSyslogConfiguration *SysLog;
};
