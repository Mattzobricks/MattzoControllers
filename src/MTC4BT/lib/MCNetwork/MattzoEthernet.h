#pragma once
#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

/*
The W5000 code comes from https://github.com/PuceBaboon/ESP32_W5500_NTP_CLIENT/blob/master/src/ESP32_NTP.ino
*/
// standard pins for SPI are used, the next ones are specific
#define RESET_P 26
#define CS_P 5

/*  In pins_arduino.h the following pins are defined, we are using MOSI, MISO and SCK
	We are not using the INT pin on the WS500 module.
static const uint8_t SS    = 5;
static const uint8_t MOSI  = 23;
static const uint8_t MISO  = 19;
static const uint8_t SCK   = 18;
*/

/*
 * Wiz W5500 reset function.  Change this for the specific reset
 * sequence required for your particular board or module.
 */
void WizReset();

/*
 * This is a crock. It's here in an effort
 * to help people debug hardware problems with
 * their W5100 ~ W5500 board setups.  It's
 * a copy of the Ethernet library enums and
 * should, at the very least, be regenerated
 * from Ethernet.h automatically before the
 * compile starts (that's a TODO item).
 *
 */
/*
 * Print the result of the hardware status enum
 * as a string.
 * Ethernet.h currently contains these values:-
 *
 *  enum EthernetHardwareStatus {
 *  	EthernetNoHardware,
 *  	EthernetW5100,
 *  	EthernetW5200,
 *  	EthernetW5500
 *  };
 *
 */
void prt_hwval(uint8_t refval);

/*
 * Print the result of the ethernet connection
 * status enum as a string.
 * Ethernet.h currently contains these values:-
 *
 *  enum EthernetLinkStatus {
 *     Unknown,
 *     LinkON,
 *     LinkOFF
 *  };
 *
 */
void prt_ethval(uint8_t refval);
