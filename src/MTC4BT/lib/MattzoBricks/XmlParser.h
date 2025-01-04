#pragma once

#include <Arduino.h>

// Class used to retrieve attribute values from an XML string.
class XmlParser
{
  public:
	// Tries to read the specified string attribute from the given Xml message.
	// Returns a boolean value indicating whether the read was successful.
	static bool tryReadCharAttr(const char *xmlMessage, const char *attr, char **returnValue);

	// Tries to read the specified boolean attribute from the given Xml message.
	// Returns a boolean value indicating whether the read and parsing was successful.
	static bool tryReadBoolAttr(const char *xmlMessage, const char *attr, bool *returnValue);

	// Tries to read the specified int attribute from the given Xml message.
	// Returns a boolean value indicating whether the read and parsing was successful.
	static bool tryReadIntAttr(const char *xmlMessage, const char *attr, int *returnValue);

  private:
	static bool tryParseBool(const char *item, bool *returnValue);
	static bool tryParseInt(const char *item, int *returnValue);
};