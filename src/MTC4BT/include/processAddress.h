#pragma once

#include <string>
#include <map>

typedef struct {
    std::string name;
    std::string value;
} LUT;

class processAddress
{
  public:
	processAddress(const char *configFilePath);
	std::string process(const std::string inStr);

  protected:
    std::map<std::string, std::string> lut;
};