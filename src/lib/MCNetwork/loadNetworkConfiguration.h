#pragma once

#include "MCNetworkConfiguration.h"
#include <string.h>

MCNetworkConfiguration *loadNetworkConfiguration(const char *configFilePath)
{
    // New up a configation object, so we can set its properties.
    MCNetworkConfiguration *config = new MCNetworkConfiguration();

    // Initialize file system.
    if (!SPIFFS.begin(true)) {
        Serial.println("An error has occurred while mounting SPIFFS");
        return config;
    }

    // Read JSON network config file.
    DynamicJsonDocument doc = MCJsonConfig::ReadJsonFile(configFilePath);

    // Read logging configuration.
    MCLoggingConfiguration *logging = new MCLoggingConfiguration();

    JsonObject loggingConfig = doc["logging"];
    logging->MinLevel = loggingConfig["min_level"] | "info";

    // Read Serial log configuration
    logging->Serial = new MCLoggingSerialConfiguration();
    logging->Serial->Enabled = loggingConfig["serial"]["enabled"] | true;

    // Read Syslog configuration.
    JsonObject syslogConfig = loggingConfig["syslog"];
    logging->SysLog = new MCLoggingSyslogConfiguration();
    logging->SysLog->Enabled = syslogConfig["enabled"] | false;
    if (logging->SysLog->Enabled) {
        logging->SysLog->ServerAddress = syslogConfig["server"].as<std::string>();
        logging->SysLog->ServerPort = syslogConfig["port"] | 514;
        logging->SysLog->AppName = syslogConfig["appname"].as<std::string>();

        const char *minLevel = loggingConfig["min_level"];
        if (minLevel) {
            if (strcmp(minLevel, "debug") == 0) {
                logging->SysLog->mask = LOG_DEBUG;
            } else if (strcmp(minLevel, "info") == 0) {
                logging->SysLog->mask = LOG_INFO;
            } else if (strcmp(minLevel, "warning") == 0) {
                logging->SysLog->mask = LOG_WARNING;
            } else if (strcmp(minLevel, "error") == 0) {
                logging->SysLog->mask = LOG_ERR;
            } else if (strcmp(minLevel, "fatal") == 0) {
                logging->SysLog->mask = LOG_CRIT;
            } else {
                logging->SysLog->mask = LOG_EMERG;
            }
        }

        logging->SysLog->mask = ((1 << ((logging->SysLog->mask) + 1)) - 1);
    }

    // Attach logging configuration.
    config->Logging = logging;

    // Read WiFi configuration.
    MCWiFiConfiguration *wifi = new MCWiFiConfiguration();
    JsonObject wifiConfig = doc["wifi"];
    wifi->SSID = wifiConfig["SSID"].as<std::string>();
    wifi->password = wifiConfig["password"].as<std::string>();
    wifi->hostname = wifiConfig["hostname"].as<std::string>();
    wifi->otaPassword = wifiConfig["otaPassword"].as<std::string>();
    wifi->DailyBetweenConnectAttempsInMs = wifiConfig["wait"] | 1000;

    // Attach WiFi configuration.
    config->WiFi = wifi;

    // Read MQTT configuration.
    MCMQTTConfiguration *mqtt = new MCMQTTConfiguration();
    JsonObject mqttConfig = doc["mqtt"];
    mqtt->ServerAddress = mqttConfig["broker"].as<std::string>();
    mqtt->ServerPort = mqttConfig["port"] | 1883;
    mqtt->KeepAlive = mqttConfig["keepalive"] | 10;
    mqtt->Ping = mqttConfig["ping"] | 0;
    mqtt->Topic = "rocrail/service/command";

    // Attach MQTT configuration.
    config->MQTT = mqtt;

    // Return MCNetworkConfiguration object.
    return config;
}