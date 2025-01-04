#pragma once

#include "MCLoggingSerialConfiguration.h"
#include "MCLoggingSyslogConfiguration.h"

struct MCLoggingConfiguration {
  public:
	std::string MinLevel;
	int mask;
	MCLoggingSerialConfiguration *Serial;
	MCLoggingSyslogConfiguration *SysLog;
};
