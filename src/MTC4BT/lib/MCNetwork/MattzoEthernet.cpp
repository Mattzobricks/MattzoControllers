#include "MattzoEthernet.h"

/*
The W5000 code comes from https://github.com/PuceBaboon/ESP32_W5500_NTP_CLIENT/blob/master/src/ESP32_NTP.ino
*/
/*
 * Wiz W5500 reset function.  Change this for the specific reset
 * sequence required for your particular board or module.
 */
void WizReset()
{
	Serial.print("Resetting Wiz W5500 Ethernet Board...  ");
	pinMode(RESET_P, OUTPUT);
	digitalWrite(RESET_P, HIGH);
	delay(250);
	digitalWrite(RESET_P, LOW);
	delay(50);
	digitalWrite(RESET_P, HIGH);
	delay(350);
	Serial.println("Done.");
}

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
void prt_hwval(uint8_t refval)
{
	switch (refval) {
	case 0:
		Serial.println("No hardware detected.");
		break;
	case 1:
		Serial.println("WizNet W5100 detected.");
		break;
	case 2:
		Serial.println("WizNet W5200 detected.");
		break;
	case 3:
		Serial.println("WizNet W5500 detected.");
		break;
	default:
		Serial.println("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
	}
}

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
void prt_ethval(uint8_t refval)
{
	switch (refval) {
	case 0:
		Serial.println("Uknown status.");
		break;
	case 1:
		Serial.println("Link flagged as UP.");
		break;
	case 2:
		Serial.println("Link flagged as DOWN. Check cable connection.");
		break;
	default:
		Serial.println("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
	}
}
