#include "MCJsonConfig.h"

JsonDocument MCJsonConfig::ReadJsonFile(const char *jsonFilePath)
{
	// Allocate a temporary JsonDocument.
	// Don't forget to change the capacity to match your requirements!
	// Use https://arduinojson.org/v6/assistant to compute the capacity.
	// Use the settings:
	// - Processor: ESP32
	// - Mode: Deserialize
	// - Input type: Stream
	// Including some slack (1024) in case the strings change, and rounded to a power of two.
	JsonDocument doc;

	// Check if file exists.
	File file = SPIFFS.open(jsonFilePath);
	if (!file) {
		Serial.println("Config: Failed to open file for reading");
		return doc;
	}

	// Check if file is empty.
	size_t size = file.size();
	if (!size) {
		file.close();
		Serial.println("Config: File is empty");
		return doc;
	}

	// Deserialize the JSON document.
	DeserializationError error = deserializeJson(doc, file);
	if (error) {
		Serial.print(F("Config: Failed to read config file: deserializeJson() failed with code "));
		Serial.println(error.c_str());
	}

	// Close the file.
	file.close();

	return doc;
}