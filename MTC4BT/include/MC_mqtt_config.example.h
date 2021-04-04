#pragma once

#include <Arduino.h>

// MQTT server and port.
#define MQTT_BROKER_IP "1.2.3.4"
#define MQTT_BROKER_PORT 1883

// MQTT keep alive interval (in seconds).
#define MQTT_KEEP_ALIVE_INTERVAL 10

// MQTT task priority.
#define MQTT_TASK_PRIORITY 1

// MQTT handle message task core ID.
#define MQTT_HANDLE_MESSAGE_TASK_COREID 1

// The size of the MQTT task stack specified as the number of bytes.
#define MQTT_TASK_STACK_DEPTH 2048

// Number of message received the MQTT queue can hold before we start dropping them.
// Please note the more messages allowed to be queued, the more (heap) memory we consume.
#define MQTT_INCOMING_QUEUE_LENGTH 100

// Number of message to send the MQTT queue can hold before we start dropping them.
// Please note the more messages allowed to be queued, the more (heap) memory we consume.
#define MQTT_OUTGOING_QUEUE_LENGTH 1000

#define ROCRAIL_COMMAND_TOPIC "rocrail/service/command"

// *************
// Ping settings
// *************
// Attention: pings were deprecated with issue #9 and replaced by MQTT last will messages.
// Set to true if pings should be sent.
#define SEND_PING false

// Interval for sending pings in milliseconds (5000 ^= 5 seconds).
#define SEND_PING_INTERVAL 5000