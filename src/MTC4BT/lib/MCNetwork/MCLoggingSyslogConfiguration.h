#pragma once

#include <Arduino.h>

struct MCLoggingSyslogConfiguration {
	bool Enabled;
	std::string ServerAddress;
	uint16_t ServerPort;
	std::string AppName;
	int mask;
};