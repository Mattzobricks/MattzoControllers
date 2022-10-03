/*
 * General header file
 */

#include <MattzoController_Library.h>
/*
 Variables that MUST be declared in the MTC4PF_conf_<loco>.h, otherwise a link/compilation error will occurr
*/
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* MQTT_BROKER_IP;
extern const int MQTT_BROKER_PORT;
extern const int MQTT_KEEP_ALIVE_INTERVAL;
extern const int LOGLEVEL_SERIAL;
extern const int LOGLEVEL_SYSLOG;
extern const bool SYSLOG_ENABLED;
extern const char* SYSLOG_SERVER;
extern const int SYSLOG_PORT;
extern const char* OTA_PASSWORD;

extern const uint8_t IR_LED_PIN;
extern const uint8_t STATUS_LED_PIN;
extern const bool STATUS_LED_PIN_INSTALLED;
extern const bool STATUS_LED_REVERSE;
extern const bool TRIGGER_EBREAK_UPON_DISCONNECT;
extern const char* MC_HOSTNAME;
extern const char* SYSLOG_APP_NAME;