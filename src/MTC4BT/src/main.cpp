#include <Arduino.h>

#include "MCJsonConfig.h"
#include "MCmqtt.h"
#include "MTC4BTController.h"
#include "MTC4BTMQTTHandler.h"
#include "log4MC.h"

#include "processAddress.h"

#include "loadControllerConfiguration.h"
#include "loadNetworkConfiguration.h"

#include "rocrailitems/lclist.h"

// Globals
MCNetworkConfiguration *networkConfig;
MTC4BTController *controller;
MTC4BTConfiguration *controllerConfig;

#define NETWORK_CONFIG_FILE "/network_config.json"
#define CONTROLLER_CONFIG_FILE "/controller_config.json"
#define LOOKUPTABLE_CONFIG_FILE "/lookuptable_config.json"

WiFiClient wifiClient;
PubSubClient client(wifiClient);
bool gotConnection = false;
bool gotMQTTConnection = false;

/******************************************************************************************************************************
 * MQTT Code, moved here, maybe later move to separate files
 *
 */
void (*handler)(const char *message) = MTC4BTMQTTHandler::Handle;
void (*infohandler)(const char *message) = MTC4BTMQTTHandler::infoHandle;

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
	payload[length - 1] = 0;
	// log4MC::vlogf(LOG_INFO, "MQTT: Received on '%s' command. (%s)", topic, (char * )payload );
	if (handler && strstr(topic, MQTT_COMMANDTOPIC) != nullptr) // shouldn't be null, but in just in case it is do not crash!
		(*handler)((char *)payload);
	if (infohandler && strstr(topic, MQTT_INFOTOPIC) != nullptr) // shouldn't be null, but in just in case it is do not crash!
		(*infohandler)((char *)payload);
}

long lastReconnectAttempt = 0;
boolean reconnect()
{
	char request[251];
	snprintf(request, 250, "<sys cmd=\"ebreak\" source=\"lastwill\" mc=\"%s\"/>", networkConfig->MQTT->SubscriberName);
	if (client.connect(networkConfig->MQTT->SubscriberName, MQTT_COMMANDTOPIC, 0, false, request)) {
		log4MC::vlogf(LOG_INFO, "MQTT: Connected %s", networkConfig->MQTT->SubscriberName);
		client.subscribe(MQTT_COMMANDTOPIC);
		client.subscribe(MQTT_INFOTOPIC);
	} else {
		log4MC::vlogf(LOG_WARNING, "MQTT: Subscriber connect failed, rc=%d. Try again in a few seconds...", client.state());
	}

	return client.connected();
}
/******************************************************************************************************************************
 * WiFi code
 *
 */

// WARNING: This function is called from a separate FreeRTOS task (thread)!
void WiFiEvent(WiFiEvent_t event)
{
	log4MC::vlogf(LOG_DEBUG, "[WiFi-event] event: %d\n", event);

	switch (event) {
	case ARDUINO_EVENT_WIFI_READY:
		log4MC::debug("WiFi interface ready");
		break;
	case ARDUINO_EVENT_WIFI_SCAN_DONE:
		log4MC::debug("Completed scan for access points");
		break;
	case ARDUINO_EVENT_WIFI_STA_START:
		log4MC::debug("WiFi client started");
		break;
	case ARDUINO_EVENT_WIFI_STA_STOP:
		log4MC::debug("WiFi clients stopped");
		break;
	case ARDUINO_EVENT_WIFI_STA_CONNECTED:
		log4MC::debug("Connected to access point");
		break;
	case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
		log4MC::debug("Disconnected from WiFi access point");
		break;
	case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
		log4MC::debug("Authentication mode of access point has changed");
		break;
	case ARDUINO_EVENT_WIFI_STA_GOT_IP:
		Serial.print("Obtained WiFi IP address: ");
		Serial.println(WiFi.localIP());
		gotConnection = true;
		break;
	case ARDUINO_EVENT_WIFI_STA_LOST_IP:
		log4MC::debug("Lost IP address and IP address is reset to 0");
		break;
	case ARDUINO_EVENT_WPS_ER_SUCCESS:
		log4MC::debug("WiFi Protected Setup (WPS): succeeded in enrollee mode");
		break;
	case ARDUINO_EVENT_WPS_ER_FAILED:
		log4MC::debug("WiFi Protected Setup (WPS): failed in enrollee mode");
		break;
	case ARDUINO_EVENT_WPS_ER_TIMEOUT:
		log4MC::debug("WiFi Protected Setup (WPS): timeout in enrollee mode");
		break;
	case ARDUINO_EVENT_WPS_ER_PIN:
		log4MC::debug("WiFi Protected Setup (WPS): pin code in enrollee mode");
		break;
	case ARDUINO_EVENT_WIFI_AP_START:
		log4MC::debug("WiFi access point started");
		break;
	case ARDUINO_EVENT_WIFI_AP_STOP:
		log4MC::debug("WiFi access point  stopped");
		break;
	case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
		log4MC::debug("Client connected");
		break;
	case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
		log4MC::debug("Client disconnected");
		break;
	case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
		log4MC::debug("Assigned IP address to client");
		break;
	case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
		log4MC::debug("Received probe request");
		break;
	case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
		log4MC::debug("AP IPv6 is preferred");
		break;
	case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
		log4MC::debug("STA IPv6 is preferred");
		break;
	case ARDUINO_EVENT_ETH_GOT_IP6:
		log4MC::debug("Ethernet IPv6 is preferred");
		break;
	case ARDUINO_EVENT_ETH_START:
		log4MC::debug("Ethernet started");
		ETH2.setHostname(networkConfig->hostname.c_str());
		break;
	case ARDUINO_EVENT_ETH_STOP:
		log4MC::debug("Ethernet stopped");
		break;
	case ARDUINO_EVENT_ETH_CONNECTED:
		log4MC::debug("Ethernet connected");
		break;
	case ARDUINO_EVENT_ETH_DISCONNECTED:
		log4MC::debug("Ethernet disconnected");
		gotConnection = false;
		break;
	case ARDUINO_EVENT_ETH_GOT_IP:
		Serial.print("Obtained ETH IP address: ");
		Serial.println(ETH2.localIP());
		gotConnection = true;
		break;
	default:
		log4MC::debug("Unknown event");
		break;
	}
}

// WARNING: This function is called from a separate FreeRTOS task (thread)!
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
	Serial.printf(" MAC Address: %s\n", WiFi.macAddress().c_str());

	gotConnection = true;
}

void ETHGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
	Serial.println("ETH connected");
	Serial.println("IP address: ");
	Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
	Serial.printf(" MAC Address: %s\n", ETH.macAddress().c_str());

	gotConnection = true;
}

/******************************************************************************************************************************
 * Rest of the code
 */

#ifdef ESP32
// 1 minute, 30, 15 or 10 seconds
#ifdef TICKER
#if TICKER == 1 or TICKER == 2 or TICKER == 4 or TICKER == 6
#define SETUPTICKER
void handleTickerLoop(void *param)
{
	long minuteTicker = 0;
	unsigned long timeTaken = 0;
	for (;;) {
		timeTaken = millis();
		log4MC::vlogf(LOG_INFO, "Minutes uptime: %d.%02d", (minuteTicker / TICKER), (minuteTicker % TICKER) * (60 / TICKER));
		log4MC::vlogf(LOG_INFO, "  Memory Heap free: %8u max alloc: %8u min free: %8u", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
		minuteTicker++;
		timeTaken = abs((long)(timeTaken - millis()));
		delay(60000 / TICKER - timeTaken);
	}
}
void setupTicker()
{
	xTaskCreatePinnedToCore(handleTickerLoop, "TickerHandler", 2048, NULL, 2, NULL, 1);
	delay(500);
}
#else
#error "Invalid ticker value, valid values are 1,2,4 or 6"
#endif
#endif
#endif

void setup()
{
	// Configure Serial.
	Serial.begin(115200);

	// Wait a moment to start (so we don't miss Serial output).
	delay(1000 / portTICK_PERIOD_MS);

	Serial.println();
	Serial.println("[" + String(xPortGetCoreID()) + "] Setup: Starting MTC4BT...");

	// Load the network configuration.
	Serial.println("[" + String(xPortGetCoreID()) + "] Setup: Loading network configuration...");
	networkConfig = loadNetworkConfiguration(NETWORK_CONFIG_FILE);

	// DEPRICATION WARNING, hostname and otaPassword are gone in the one of the next releases and needs to be in the network part
	if (networkConfig->WiFi->hostname != "") {
		Serial.println("Config: DEPRECATION WARNING, found the \"hostname\" field in the \"wifi\" part of the config, this needs to be moved to the \"network\" part if the config");
		networkConfig->hostname = networkConfig->WiFi->hostname;
	}
	if (networkConfig->WiFi->otaPassword != "") {
		Serial.println("Config: DEPRECATION WARNING, found the \"otaPassword\" field in the \"wifi\" part of the config, this needs to be moved to the \"network\" part if the config");
		networkConfig->otaPassword = networkConfig->WiFi->otaPassword;
	}

	// Setup logging (from now on we can use log4MC).
	log4MC::Setup(networkConfig->hostname.c_str(), networkConfig->Logging);

	// Load optional lookuptable
	processAddress * processor = new processAddress(LOOKUPTABLE_CONFIG_FILE);

	// Load the controller configuration.
	log4MC::info("Setup: Loading controller configuration...");
	controllerConfig = loadControllerConfiguration(CONTROLLER_CONFIG_FILE , processor);
	controller = new MTC4BTController();
	controller->Setup(controllerConfig);
	networkConfig->MQTT->SubscriberName = controllerConfig->ControllerName;

	log4MC::info("Setup: Controller configuration completed.");

	controller->setStatusLedInSetup(100); // led on

	WiFi.disconnect(true);
	delay(1000);

	// Examples of different ways to register wifi events;
	// these handlers will be called from another thread.
	WiFi.onEvent(WiFiEvent);
	WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
	WiFi.onEvent(ETHGotIP, WiFiEvent_t::ARDUINO_EVENT_ETH_GOT_IP);
	WiFiEventId_t eventID = WiFi.onEvent(
		[](WiFiEvent_t event, WiFiEventInfo_t info) {
			Serial.print("WiFi lost connection. Reason: ");
			Serial.println(info.wifi_sta_disconnected.reason);
		},
		WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
	WiFiEventId_t eventID2 = WiFi.onEvent(
		[](WiFiEvent_t event, WiFiEventInfo_t info) {
			Serial.println("ETH lost connection.");
		},
		WiFiEvent_t::ARDUINO_EVENT_ETH_DISCONNECTED);
	// Remove WiFi event
	log4MC::vlogf(LOG_INFO, "setup: WiFi Event ID: %d", eventID);

	WiFi.disconnect(true);
	WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
	WiFi.setHostname(networkConfig->hostname.c_str());
	if (networkConfig->networkType == "wireless") {
		log4MC::vlogf(LOG_INFO, "Wifi: Connecting to %s.", networkConfig->WiFi->SSID.c_str());
		WiFi.begin(networkConfig->WiFi->SSID.c_str(), networkConfig->WiFi->password.c_str());
	}
#ifdef WIRED
	else if (networkConfig->networkType == "wired") {
		// the old wired interface, using those pin numbers
		/* 	WIRED_RESET_P 26
			WIRED_CS_P 5
			WIRED_INT 25
			WIRED_MOSI 23
			WIRED_MISO 19
			WIRED_SCK 18
		*/
		if (!ETH.begin(ETH_PHY_W5500, 1, 5, 25, 26,
					   SPI3_HOST,
					   18, 19, 23)) {
			// wired connection failed
			log4MC::error(" \"wired\" hardware fault, or cable problem... cannot continue.");
		}
	}
#endif

	log4MC::info("Wait for WiFi/ETH... ");

	ArduinoOTA.setHostname(networkConfig->hostname.c_str());
	ArduinoOTA.setPassword(networkConfig->otaPassword.c_str());

	ArduinoOTA
		.onStart([]() {
			String type;
			if (ArduinoOTA.getCommand() == U_FLASH)
				type = "sketch";
			else // U_SPIFFS
				type = "filesystem";

			// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
			Serial.println("Start updating " + type);
		})
		.onEnd([]() {
			Serial.println("\nEnd");
		})
		.onProgress([](unsigned int progress, unsigned int total) {
			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		})
		.onError([](ota_error_t error) {
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

	log4MC::vlogf(LOG_INFO, "MQTT: Connecting to %s:%u...", networkConfig->MQTT->ServerAddress.c_str(), networkConfig->MQTT->ServerPort);

	client.setServer(networkConfig->MQTT->ServerAddress.c_str(), networkConfig->MQTT->ServerPort);
	client.setKeepAlive(networkConfig->MQTT->KeepAlive);
	client.setBufferSize(MAXBUFFERSIZE);
	client.setCallback(mqttCallback);

	delay(1500);
	lastReconnectAttempt = 0;
	// controller->setStatusLedInSetup(0); // led off

	// all network stuff is done, start the scanner
	controller->SetupScanner();

	log4MC::info("Setup: MattzoTrainController for BLE running.");

	if (controllerConfig->LocoConfigs.size() != 0 || controllerConfig->RemoteConfigs.size() != 0) {
		log4MC::vlogf(LOG_INFO, "Setup: Number of locos to discover hubs for: %u", controllerConfig->LocoConfigs.size());
		log4MC::vlogf(LOG_INFO, "Setup: Number of remotes to discover hubs for: %u", controllerConfig->RemoteConfigs.size());
	} else {
		log4MC::vlogf(LOG_WARNING, "No locomotives or remote found in the configuration, going into BLE scan mode.");
	}

	// stuff for the remote
	locs.reserve(10);

#ifdef ESP32
#ifdef SETUPTICKER
	setupTicker();
	log4MC::info("Ticker started");
#endif
#endif
}

void loop()
{
	static unsigned long checkedForRocrail = millis() + 12000; // force a loco lookup on startup
	static long connectionStartTime = millis();
	if (millis() - connectionStartTime > 30000 && !gotConnection) {
		log4MC::info("No connection... ");
		connectionStartTime = millis();
	}

	// only try to connect to mqtt if we have a network connection
	if (!client.connected() && gotConnection) {
		long now = millis();
		gotMQTTConnection = false;
		if (now - lastReconnectAttempt > 5000) {
			lastReconnectAttempt = now;
			// Attempt to reconnect
			log4MC::info("Try mqtt reconnect... ");
			if (reconnect()) {
				lastReconnectAttempt = 0;
				log4MC::info("Reconnected!");
				gotMQTTConnection = true;
				controller->setStatusLedInSetup(0); // led off
			}
		}
	} else {
		// Client connected

		client.loop();
	}

	controller->Loop();
	ArduinoOTA.handle();

	// Next statement is to load the loco configs into the ESP, but only if we have configured remotes
	// we test every 10 seconds for rocrail availability!
	// side effect, if there is a plan without any it will also test every 10 seconds.
	if (controllerConfig->RemoteConfigs.size() != 0) {
		// we have configured remotes
		if (locs.size() == 0) {
			if ((millis() - checkedForRocrail > 10000)) {
				// no loco's
				MTC4BTMQTTHandler::pubGetShortLcList();
				checkedForRocrail = millis();
			}
		} else {
			// we have a loco list, so there is a connection with Rocrail
			controller->initFirstItems();
		}
	}
}