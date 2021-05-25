#pragma once

#include <Arduino.h>
#include <WiFiUdp.h> // UDP library required for Syslog.
#include <Syslog.h>  // Syslog library.

#include "MCLoggingConfiguration.h"

#ifdef MC_DEBUG
#define MC_LOG_DEBUG(...) log4MC::vlogf(LOG_DEBUG, __VA_ARGS__)
#else
#define MC_LOG_DEBUG(...)
#endif

#ifdef MC_INFO
#define MC_LOG_INFO(...) log4MC::vlogf(LOG_INFO, __VA_ARGS__)
#else
#define MC_LOG_INFO(...)
#endif

class log4MC
{
public:
    // Setup the logger.
    static void Setup(const char *hostName, MCLoggingConfiguration *config);
    static void wifiIsConnected(bool connected);
    static void vlogf(uint8_t level, const char *fmt, ...);
    static void log(uint8_t level, const char *message);
    static void debug(const char *message);
    static void info(const char *message);
    static void info(String message);
    static void warn(const char *message);
    static void error(const char *message);
    static void fatal(const char *message);

private:
    static void logMessage(uint8_t level, char *message);
    static void setLogMask(uint8_t priMask);
    static uint8_t getLogMask();

    static MCLoggingConfiguration *_config;
    static bool _connected;
    static uint8_t _priMask;
    static WiFiUDP _udpClient;
    static Syslog syslog;
};
