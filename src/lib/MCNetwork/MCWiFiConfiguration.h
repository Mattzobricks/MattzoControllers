#pragma once

struct MCWiFiConfiguration {
  public:
    std::string SSID;
    std::string password;
    std::string otaPassword;
    std::string hostname;
    uint32_t DailyBetweenConnectAttempsInMs;
};