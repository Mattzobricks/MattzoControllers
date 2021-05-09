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

// Blocking call waiting for a WiFi connection. It also handles OTA updates.
void MattzoWifiClient::Assert()
{
  if (!_setupCompleted)
  {
    Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Setup not completed. Execute .Setup() first.");
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
      Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Connection " + _config->SSID.c_str() + " lost. Reconnecting...");
    }

    // Re-test connection ater a small delay.
    delay(_config->DailyBetweenConnectAttempsInMs);
  }

  log4MC::wifiIsConnected(true);

  if (!_wasConnected)
  {
    _wasConnected = true;
    log4MC::vlogf(LOG_INFO,"Wifi: Connected (IPv4: %s).", WiFi.localIP().toString().c_str());
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

    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}

// Initialize private static members.
MCWiFiConfiguration *MattzoWifiClient::_config;
bool MattzoWifiClient::_setupInitiated = false;
bool MattzoWifiClient::_setupCompleted = false;
bool MattzoWifiClient::_wasConnected = false;
