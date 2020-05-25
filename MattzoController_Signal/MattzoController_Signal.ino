#include <EEPROM.h>  // EEPROM library
#include <ESP8266WiFi.h>  // WiFi library
#include <PubSubClient.h>  // MQTT library
#include <tinyxml2.h>  // tiny xml 2 library

using namespace tinyxml2;

String eepromIDString = "MattzoSignalController";  // ID String. If found in EEPROM, the controller id is deemed to be set and used by the controller; if not, a random controller id is generated and stored in EEPROM memory
const int eepromIDStringLength = 22;  // length of the ID String. Needs to be updated if the ID String is changed.
unsigned int controllerNo;  // controllerNo. Read from memory upon starting the controller. Ranges between 1 and MAX_CONTROLLER_ID.
const int MAX_CONTROLLER_ID = 65000;

const char* SSID = "TBW13";
const char* PSK = "tbw13iscool";
const char* MQTT_BROKER = "192.168.1.19";
String mqttClientName;
char mqttClientName_char[eepromIDStringLength + 5 + 1];  // the name of the client must be given as char[]. Length must be the ID String plus 5 figures for the controller ID.

const int NUM_SIGNALPORTS = 8; // Number of signal ports. Each port corresponds to one light of a signal. E.g., a light with one red and one green light consumes two ports.
uint8_t SIGNALPORT_PIN[NUM_SIGNALPORTS];  // Digital PINs for output

const int TICKS_BETWEEN_PINGS = 500;  // number of ticks after which the sensor will send a ping via MQTT. 500 = 5 seconds.
int ticksBetweenPingsCounter = 0;     // tick counter

WiFiClient espClient;
PubSubClient client(espClient);

// signals
const int signal1greenLED = D0;
const int signal1redLED = D1;
int signal1State = 2;
const int signal2greenLED = D2;
const int signal2redLED = D3;
int signal2State = 2;
const int signal3greenLED = D4;
const int signal3redLED = D5;
int signal3State = 2;
const int signal4greenLED = D6;
const int signal4redLED = D7;
int signal4State = 2;

void setup() {
  SIGNALPORT_PIN[0] = D1;
  SIGNALPORT_PIN[1] = D2;
  SIGNALPORT_PIN[2] = D3;
  SIGNALPORT_PIN[3] = D4;
  SIGNALPORT_PIN[4] = D5;
  SIGNALPORT_PIN[5] = D6;
  SIGNALPORT_PIN[6] = D7;
  SIGNALPORT_PIN[7] = D8;

  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < NUM_SIGNALPORTS; i++) {
	  pinMode(i, OUTPUT);
  }

  // a short blink to say "hello, I have power supply and booting up"
  for (int j = 0; j < 3; j++) {
    allBlink(true, 0);
    delay(200);
    allBlink(false, 0);
    delay(400);
  }

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
    allBlink(true, 0);
    delay(500);
    allBlink(false, 0);
    delay(500);
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

void allBlink(boolean onOff, int blinkType) {
  if (blinkType == 0) {
    setLED(0, onOff);
    setLED(1, onOff);
    setLED(2, onOff);
    setLED(3, onOff);
    setLED(4, onOff);
    setLED(5, onOff);
    setLED(6, onOff);
    setLED(7, onOff);
  } else if (blinkType == 1) {
    setLED(0, onOff);
    setLED(1, !onOff);
    setLED(2, onOff);
    setLED(3, !onOff);
    setLED(4, onOff);
    setLED(5, !onOff);
    setLED(6, onOff);
    setLED(7, !onOff);
  }
  
  return;

  // TODO: Checken, wieso das folgende nicht funktioniert!
  
  if (onOff) {
    for (int i=0; i < NUM_SENSORS; i++) {
      if (blinkType == 0) {
        // all flash
        setLED(i, onOff);
      } else {
        // flash alternatively
        setLED(i, onOff ^ (i % 2 == 0));
      }
    }
  }
}

void setLED(int index, bool ledState) {
  if (ledState) {
    digitalWrite(SIGNALPORT_PIN[index], LOW);
  } else {
    digitalWrite(SIGNALPORT_PIN[index], HIGH);
  }
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
    return;
  }

  Serial.println("Parsing XML successful");
  XMLElement * element = xmlDocument.FirstChildElement("co");
  if (element == NULL) {
    Serial.println("<co> node not found. Message disregarded.");
    return;
  }

  Serial.println("<co> node found.");

  // query addr1 attribute. This is the MattzoController id.
  // If this does not equal the ControllerNo of this controller, the message is disregarded.
  int rr_addr1 = 0;
  if (element->QueryIntAttribute("addr1", &rr_addr1) != XML_SUCCESS) {
    Serial.println("addr1 attribute not found or wrong type. Message disregarded.");
    return;
  }
  Serial.println("addr1: " + String(rr_addr1));
  if (rr_addr1 != controllerNo) {
    Serial.println("Message disgarded, as it is not for me (" + String(controllerNo) + ")");
    return;
  }

  // query port1 attribute. This is port id of the port to which the switch is connected.
  // If the controller does not have such a port, the message is disregarded.
  int rr_port1 = 0;
  if (element->QueryIntAttribute("port1", &rr_port1) != XML_SUCCESS) {
    Serial.println("port1 attribute not found or wrong type. Message disregarded.");
    return;
  }
  Serial.println("port1: " + String(rr_port1));
  if (rr_port1 < 1 || rr_port1 > NUM_SIGNALPORTS) {
    Serial.println("Message disgarded, as this controller does not have such a port.");
    return;
  }

  Serial.println("Port: " + String(rr_port1));
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

  // Send ping?
  if (++ticksBetweenPingsCounter > TICKS_BETWEEN_PINGS) {
    ticksBetweenPingsCounter = 0;
    Serial.println("Sending PING!");
    client.publish("roc2bricks/ping", mqttClientName_char);
  }

  delay(10);
}
