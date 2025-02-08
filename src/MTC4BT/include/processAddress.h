#pragma once

#include <string>

class processAddress
{
  public:
	processAddress(const char *configFilePath);
	std::string process(const std::string inStr);
};