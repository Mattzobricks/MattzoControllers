#pragma once
#include "ETHClass2.h" //Is to use the modified ETHClass
#define ETH ETH2
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SPI.h>


#define MQTT_COMMANDTOPIC "rocrail/service/command"
#define MQTT_CLIENTTOPIC "rocrail/service/client"
#define MQTT_INFOTOPIC "rocrail/service/info"

#define MAXBUFFERSIZE  16384

extern WiFiClient wifiClient;
extern PubSubClient client;

extern bool gotConnection;
extern bool gotMQTTConnection;