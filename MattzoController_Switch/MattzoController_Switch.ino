#include <EEPROM.h>  // EEPROM library
#include <ESP8266WiFi.h>  // WiFi library
#include <PubSubClient.h>  // MQTT library
#include <Servo.h>  // servo library
#include <tinyxml2.h>  // tiny xml 2 library

using namespace tinyxml2;

String eepromIDString = "MattzoSwitchController";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 22;  // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerNo;  // controllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 65000;

const char* SSID = "TBW13";
const char* PSK = "tbw13iscool";
const char* MQTT_BROKER = "192.168.1.19";
String mqttClientName;
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

const int NUM_SWITCHPORTS = 8; // Number of switch ports
uint8_t SWITCHPORT_PIN[NUM_SWITCHPORTS];  // Digital PINs for output

WiFiClient espClient;
PubSubClient client(espClient);

// create servo objects to control servos
Servo servo[NUM_SWITCHPORTS];

// Default values for TrixBrix switches (in case servo angles are not transmitted)
const int SERVO_MIN = 70;
const int SERVO_MAX = 85;

// delay between to switch operations
const int SWITCH_DELAY = 200;



void setup() {
    Serial.begin(115200);
    randomSeed(ESP.getCycleCount());
    Serial.println("");
    Serial.println("MattzoController booting...");

    servo[0].attach(D1);
    servo[1].attach(D2);
    servo[2].attach(D3);
    servo[3].attach(D4);
    servo[4].attach(D5);
    servo[5].attach(D6);
    servo[6].attach(D7);
    servo[7].attach(D8);

    loadPreferences();
    setup_wifi();
    setup_mqtt();
}

void loadPreferences() {
  int i;
  int controllerNoHiByte;
  int controllerNoLowByte;

  // set-up EEPROM read/write operations
  EEPROM.begin(512);

  // Check if the first part of the memory is filled with the MattzoController ID string.
  // This is the case if the controller has booted before with a MattzoController firmware.
  bool idStringCheck = true;
  for (i = 0; i < eepromIDString.length(); i++) {
    char charEeprom = EEPROM.read(i);
    char charIDString = eepromIDString.charAt(i);
    if (charEeprom != charIDString) {
      idStringCheck = false;
      break;
    }
  }

  // TODO: also write / read SSID, Wifi-Password and MQTT Server from EEPROM

  int paramsStartingPosition = eepromIDString.length();
  if (idStringCheck) {
    // load controller number from preferences
    controllerNoHiByte = EEPROM.read(paramsStartingPosition);
    controllerNoLowByte = EEPROM.read(paramsStartingPosition + 1);
    controllerNo = controllerNoHiByte * 256 + controllerNoLowByte;
    Serial.println("Loaded controllerNo from EEPROM: " + String(controllerNo));

  } else {
    // preferences not initialized yet -> initialize controller
    // this runs only a single time when starting the controller for the first time

    // Wait a bit to give the user some time to open the serial console...
    delay (5000);
    
    Serial.println("Initializing controller preferences on first start-up...");
    for (i = 0; i < eepromIDString.length(); i++) {
      EEPROM.write(i, eepromIDString.charAt(i));
    }

    // assign random controller number between 1 and 65000 and store in EEPROM
    controllerNo = random(1, MAX_CONTROLLER_ID);
    controllerNoHiByte = controllerNo / 256;
    controllerNoLowByte = controllerNo % 256;
    EEPROM.write(paramsStartingPosition, controllerNoHiByte);
    EEPROM.write(paramsStartingPosition + 1, controllerNoLowByte);

    // Commit EEPROM write operation
    EEPROM.commit();

    Serial.println("Assigned random controller no " + String(controllerNo) + " and stored to EEPROM");
  }

  // set MQTT client name
  mqttClientName = eepromIDString + String(controllerNo);
  mqttClientName.toCharArray(mqttClientName_char, mqttClientName.length() + 1);
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(SSID);
 
    WiFi.begin(SSID, PSK);
 
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");

      // TODO: Support WPS! Store found Wifi network found via WPS in EEPROM and use next time!
    }
 
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
 
void setup_mqtt() {
    client.setServer(MQTT_BROKER, 1883);
    client.setCallback(callback);
    client.setBufferSize(2048);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length+1];
  for (int i = 0; i < length; i++) {
      // Serial.print((char)payload[i]);
      msg[i] = (char)payload[i];
  }
  // Serial.println();

  msg[length] = '\0';
  Serial.println(msg);

  XMLDocument xmlDocument;
  if(xmlDocument.Parse(msg)!= XML_SUCCESS){
    Serial.println("Error parsing");
  } else {
    Serial.println("Parsing successful");
    XMLElement * element = xmlDocument.FirstChildElement("sw");
    if (element != NULL) {
      Serial.println("sw node found");
      const XMLAttribute * attribute_addr1 = element->FindAttribute("addr1");
      if (attribute_addr1) {
        int rr_addr1 = attribute_addr1->Value();
        Serial.print("addr1 attribute found, value ");
        Serial.println(attribute_addr1->Value());
        const XMLAttribute * attribute_port1 = element->FindAttribute("port1");
        if (attribute_port1) {
          Serial.print("port1 attribute found, value ");
          Serial.println(attribute_port1->Value());
        } else {
          Serial.println("port1 attribute not found");
        }
      } else {
        Serial.println("attribute addr1 not found");
      }
    } else {
      Serial.println("no sw node found");
    }
  }

  if(strcmp(msg, "T|Switch 2|s")==0) {
    Serial.println("Switch 2 straight");
    servo[1].write(SERVO_MAX);
    delay(SWITCH_DELAY);
  }
  else if(strcmp(msg, "T|Switch 2|t")==0) {
    Serial.println("Switch 2 turn");
    servo[1].write(SERVO_MIN);
    delay(SWITCH_DELAY);
  }
}

void reconnect() {
    while (!client.connected()) {
        Serial.println("Reconnecting MQTT...");
        if (!client.connect(mqttClientName_char)) {
          Serial.print("Failed, rc=");
          Serial.print(client.state());
          Serial.println(". Retrying in 5 seconds...");
          delay(5000);
        }
    }
    client.subscribe("rocrail/service/command");
    Serial.println("MQTT Connected.");
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}
