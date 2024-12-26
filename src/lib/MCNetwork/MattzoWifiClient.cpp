#include <ArduinoOTA.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>

#include "MCFS.h"
#include "MattzoWifiClient.h"
#include "log4MC.h"

#include "MattzoEthernet.h"
/*
W5500 support is compiled in, with a -D flag.
If compiled in, it will search for an ethernet module over SPI with also _D defined pins. If found it will try and connect, if the LinkOff, then and only then it will try to use wifi, otherwise it will use wired network.
This has the least impact on the current code, in regards to the configuration.
If we want it configurable in the json it has more impact.
for testing purposes I will first implement the the first option and see how it works.

The W5000 code comes from https://github.com/PuceBaboon/ESP32_W5500_NTP_CLIENT/blob/master/src/ESP32_NTP.ino
*/
void MattzoWifiClient::Loop()
{
#ifdef WIRED
    if (!useWifiStatus) { // if we are on the wire
        // You only need to call maintain if you're using DHCP.
        Ethernet.maintain();
        if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("Nolink");
            delay(250);
        }
    }
#endif
    ArduinoOTA.handle();
}

void MattzoWifiClient::Setup(MCWiFiConfiguration *config)
{
    _config = config;
#ifdef WIRED
    log4MC::vlogf(LOG_INFO, "Wifi: All wifi log lines may also be Wired!");
#endif

    if (_setupCompleted) {
        log4MC::vlogf(LOG_WARNING, "Wifi: Setup already completed!");
        return;
    }

    if (_setupInitiated) {
        Serial.println("[" + String(xPortGetCoreID()) + "] Wifi: Setup already initiated!");
        return;
    } else {
        _setupInitiated = true;
    }
#ifdef WIRED
    // First go for wired, if this is not working, fallback to Wifi
    // Variable to store the MAC address
    uint8_t baseMac[6];

    esp_read_mac(baseMac, ESP_MAC_ETH);
    Serial.print("Ethernet MAC: ");
    for (int i = 0; i < 5; i++) {
        Serial.printf("%02X:", baseMac[i]);
    }
    Serial.printf("%02X\n", baseMac[5]);
    // Use Ethernet.init(pin) to configure the CS pin.
    Ethernet.init(CS_P); // GPIO5 on the ESP32.
    WizReset();
    Serial.println("Starting ETHERNET connection...");
    Ethernet.begin(baseMac);

    delay(200);

    Serial.print("Ethernet IP is: ");
    Serial.println(Ethernet.localIP());
    /*
     * Sanity checks for W5500 and cable connection.
     */
    Serial.print("Checking connection.");
    bool rdy_flag = false;
    for (uint8_t i = 0; i <= 20; i++) {
        if ((Ethernet.hardwareStatus() == EthernetNoHardware) || (Ethernet.linkStatus() == LinkOFF)) {
            Serial.print(".");
            rdy_flag = false;
            delay(80);
        } else {
            rdy_flag = true;
            break;
        }
    }
    if (rdy_flag == false) {
        Serial.println("\n\r\tHardware fault, or cable problem... cannot continue.");
        Serial.print("Hardware Status: ");
        prt_hwval(Ethernet.hardwareStatus());
        Serial.print("   Cable Status: ");
        prt_ethval(Ethernet.linkStatus());
        Serial.println("  Fallback to wifi ");
    } else {
        Serial.println(" OK");
        Serial.println();

        // startOTA(); NO OTA FOR WIRED!!!!
        // Setup completed.
        _setupCompleted = true;
        useWifiStatus = false;
        return;
    }
#endif

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
    while (WiFi.status() != WL_CONNECTED) {
        delay(_config->DailyBetweenConnectAttempsInMs);
        Serial.print(".");
    }

    Serial.println();

    Serial.print("Wifi IP is: ");
    Serial.println(WiFi.localIP());
    startOTA();
    // Setup completed.
    _setupCompleted = true;
}

int MattzoWifiClient::status()
{
#ifndef WIRED
    // force wifi
    useWifiStatus = true;
#endif
    if (useWifiStatus) {
        return WiFi.status();
    } else {
        return Ethernet.linkStatus();
    }
}

int MattzoWifiClient::GetStatus()
{
    if (!_setupInitiated) {
        return WL_UNINITIALIZED;
    } else if (!_setupCompleted) {
        return WL_INITIALIZING;
    } else {
        return status();
    }
}

// Blocking call waiting for a WiFi connection. It also handles OTA updates.
void MattzoWifiClient::Assert()
{
    if (!_setupCompleted) {
        log4MC::error("Wifi: Setup not completed. Execute .Setup() first.");
        return;
    }

    if (status() == WL_CONNECTED || status() != LinkOFF) {
        log4MC::wifiIsConnected(true);
        _wasConnected = true;

        // Handle any OTA updates.
        ArduinoOTA.handle();
        return;
    }

    log4MC::wifiIsConnected(false);

    // Ethernet automatic reconnects, so only do wifi?
    if (useWifiStatus) {
        // Loop until we reconnect.
        while (WiFi.status() != WL_CONNECTED) {
            if (_wasConnected) {
                _wasConnected = false;
                log4MC::vlogf(LOG_WARNING, "Wifi: Connection to %s lost. Reconnecting...", _config->SSID.c_str());
                WiFi.reconnect();
            }

            // Re-test connection after a small delay.
            delay(_config->DailyBetweenConnectAttempsInMs);
        }
    }
    log4MC::wifiIsConnected(true);

    if (!_wasConnected) {
        _wasConnected = true;
        if (useWifiStatus) {
            log4MC::vlogf(LOG_INFO, "Wifi: Connected (IPv4: %s).", WiFi.localIP().toString().c_str());
        } else {
            log4MC::vlogf(LOG_INFO, "Ethernet: Connected (IPv4: %s).", Ethernet.localIP().toString().c_str());
        }
        // Start OTA listener.
        startOTA();
    }
}

void MattzoWifiClient::startOTA()
{
    ArduinoOTA.setHostname(_config->hostname.c_str());
    ArduinoOTA.setPassword(_config->otaPassword.c_str());

    log4MC::vlogf(LOG_DEBUG,"OTA Passweord '%s'.",_config->otaPassword.c_str());

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else // U_SPIFFS
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
bool MattzoWifiClient::useWifiStatus = true; // defautl wifi
