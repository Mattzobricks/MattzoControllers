#include <log4MC.h>

void log4MC::Setup(const char *hostName, MCLoggingConfiguration *config)
{
    _config = config;

    if (_config->SysLog->Enabled) {
        Serial.print("Syslog: Server: ");
        Serial.print(config->SysLog->ServerAddress.c_str());
        Serial.print(":");
        Serial.println(config->SysLog->ServerPort);

        Serial.print("Syslog: Hostname: ");
        Serial.println(hostName);

        Serial.print("Syslog: Appname: ");
        Serial.println(config->SysLog->AppName.c_str());

        syslog.server(config->SysLog->ServerAddress.c_str(), config->SysLog->ServerPort)
            .deviceHostname(hostName)
            .appName(config->SysLog->AppName.c_str())
            .defaultPriority(LOG_KERN)
            .logMask(config->SysLog->mask);

        _priMask = config->SysLog->mask;
    }

    info("Logging: Configured.");
}

void log4MC::wifiIsConnected(bool connected)
{
    _connected = connected;
}

void log4MC::logMessage(uint8_t level, char *message)
{
    if (_connected && _config->SysLog->Enabled) {
        syslog.log(level, message);
    }

    if (_config->Serial->Enabled && (LOG_MASK(level) & _priMask)) {
        Serial.println(message);
    }
}

void log4MC::setLogMask(uint8_t priMask)
{
    _priMask = priMask;
    if (_config->SysLog->Enabled) {
        syslog.logMask(_priMask);
    }
}

uint8_t log4MC::getLogMask()
{
    return _priMask;
}

void log4MC::vlogf(uint8_t level, const char *fmt, ...)
{
    char *message;
    va_list args;

    va_start(args, fmt);

    size_t initialLen;
    size_t len;

    initialLen = strlen(fmt);

    message = new char[initialLen + 11];
    len = vsnprintf(message, initialLen + 1, fmt, args);
    if (len > initialLen) {
        delete[] message;
        message = new char[len + 11];
        vsnprintf(message, len + 1, fmt, args);
    }
    va_end(args);

    log(level, message);
    delete [] message;
}

void log4MC::log(uint8_t level, const char *message)
{
    char *msg = NULL;
    unsigned int len = strlen(message);
    msg = new char[len + 20];

#ifdef ESP32
    sprintf(msg, "[%04d] [%d] %s", lineNo, xPortGetCoreID(), message);
    msg[strlen(msg)] = '\0'; // should be paded by zeros, HAVE TO TEST
#else
    sprintf(msg, "[%04d] %s", lineNo, message);
    msg[len] = '\0';
#endif
    lineNo = (lineNo +1) % 10000;
    logMessage(level, msg);
    delete [] msg;
}

void log4MC::debug(const char *message)
{
    log4MC::log(LOG_DEBUG, message);
}

void log4MC::info(const char *message)
{
    log4MC::log(LOG_INFO, message);
}

void log4MC::warn(const char *message)
{
    log4MC::log(LOG_WARNING, message);
}

void log4MC::error(const char *message)
{
    log4MC::log(LOG_ERR, message);
}

void log4MC::fatal(const char *message)
{
    log4MC::log(LOG_CRIT, message);
}

// just on string function
void log4MC::info(String message)
{
    log4MC::log(LOG_INFO, message.c_str());
}

MCLoggingConfiguration *log4MC::_config;
bool log4MC::_connected = false;
uint8_t log4MC::_priMask = LOG_MASK(LOG_INFO) | LOG_MASK(LOG_DEBUG) | LOG_MASK(LOG_WARNING) | LOG_MASK(LOG_ERR) | LOG_MASK(LOG_CRIT);
WiFiUDP log4MC::_udpClient;
Syslog log4MC::syslog(_udpClient, SYSLOG_PROTO_IETF);
unsigned int log4MC::lineNo = 0;