#include <ArduinoOTA.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>

#include "log4MC.h"
#include "MattzoWifiClient.h"
#include "MCFS.h"

void MattzoWifiClient::Setup(MCWiFiConfiguration *config)
{
  _config = config;

  if (_setupCompleted)
  {
    log4MC::vlogf(LOG_WARNING, "Wifi: Setup already completed!");
    return;
  }

  if (_setupInitiated)
  {
    Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Setup already initiated!");
    return;
  }
  else
  {
    _setupInitiated = true;
  }

#ifdef ESP8266
  WiFi.hostname(config->hostname);
#endif

  WiFi.mode(WIFI_STA);

#ifdef ESP32
  // The following code SHOULD work without the disconnect and config lines,
  // but it doesn't do its job for some ESP32s.
  // see https://github.com/espressif/arduino-esp32/issues/2537
  WiFi.disconnect(true);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(_config->hostname.c_str());
#endif

  Serial.print("[" + String(xPortGetCoreID()) + "] Wifi: Connecting to " + _config->SSID.c_str() + ".");

  WiFi.begin(_config->SSID.c_str(), config->password.c_str());

  // Loop until we actually connect.
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(_config->DailyBetweenConnectAttempsInMs);
    Serial.print(".");
  }

  Serial.println();

  log4MC::wifiIsConnected(true);
  log4MC::info("Wifi: Connected (IPv4: " + WiFi.localIP().toString() + ").");

  // Start OTA listener.
  startOTA();

  // Setup completed.
  _setupCompleted = true;
}

int MattzoWifiClient::GetStatus()
{
  if (!_setupInitiated)
  {
    return WL_UNINITIALIZED;
  }
  else if (!_setupCompleted)
  {
    return WL_INITIALIZING;
  }
  else
  {
    return WiFi.status();
  }
}

// Blocking call waiting for a WiFi connection. It also handles OTA updates.
void MattzoWifiClient::Assert()
{
  if (!_setupCompleted)
  {
    log4MC::error("Wifi: Setup not completed. Execute .Setup() first.");
    return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    log4MC::wifiIsConnected(true);

    // Handle any OTA updates.
    ArduinoOTA.handle();
    return;
  }

  log4MC::wifiIsConnected(false);

  // Loop until we reconnect.
  while (WiFi.status() != WL_CONNECTED)
  {
    if (_wasConnected)
    {
      _wasConnected = false;
      log4MC::vlogf(LOG_WARNING, "Wifi: Connection to %s lost. Reconnecting...", _config->SSID.c_str());
    }

    // Re-test connection ater a small delay.
    delay(_config->DailyBetweenConnectAttempsInMs);
  }

  log4MC::wifiIsConnected(true);

  if (!_wasConnected)
  {
    _wasConnected = true;
    log4MC::vlogf(LOG_INFO, "Wifi: Connected (IPv4: %s).", WiFi.localIP().toString().c_str());
  }
}

void MattzoWifiClient::startOTA()
{
  ArduinoOTA.setHostname(_config->hostname.c_str());
  ArduinoOTA.setPassword(_config->otaPassword.c_str());

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else // U_SPIFFS
    {
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    CONFIGFS.end(); // make sure the filesystems are unmounted

    log4MC::vlogf(LOG_INFO, "OTA: Started updating %s...", type);
  });

  ArduinoOTA.onEnd([]() {
    log4MC::info("OTA: Update completed.");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    log4MC::vlogf(LOG_INFO, "OTA: Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    log4MC::vlogf(LOG_ERR, "OTA: Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      log4MC::error("OTA: Auth failed.");
    else if (error == OTA_BEGIN_ERROR)
      log4MC::error("OTA: Begin failed.");
    else if (error == OTA_CONNECT_ERROR)
      log4MC::error("OTA: Connect failed.");
    else if (error == OTA_RECEIVE_ERROR)
      log4MC::error("OTA: Receive failed.");
    else if (error == OTA_END_ERROR)
      log4MC::error("OTA: End failed.");
  });

  ArduinoOTA.begin();
  log4MC::info("OTA: Initialized.");
}

// Initialize private static members.
MCWiFiConfiguration *MattzoWifiClient::_config;
bool MattzoWifiClient::_setupInitiated = false;
bool MattzoWifiClient::_setupCompleted = false;
bool MattzoWifiClient::_wasConnected = false;
