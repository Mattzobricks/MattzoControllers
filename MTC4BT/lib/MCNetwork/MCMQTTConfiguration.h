#pragma once

struct MCMQTTConfiguration
{
public:
    const char* SubscriberName;
    std::string ServerAddress;
    uint16_t ServerPort;
    uint16_t KeepAlive;
    uint16_t Ping;
    bool EbreakOnDisconnect;
    const char *Topic;
};