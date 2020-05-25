#include <ESP8266WiFi.h>  // WiFi library
#include <PubSubClient.h>  // MQTT library

const char* SSID = "TBW13";
const char* PSK = "tbw13iscool";
const char* MQTT_BROKER = "192.168.1.19";

WiFiClient espClient;
PubSubClient client(espClient);
// long lastMsg = 0;
// char msg[255];
// int value = 0;

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
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(signal1greenLED, OUTPUT);
  pinMode(signal1redLED, OUTPUT);
  pinMode(signal2greenLED, OUTPUT);
  pinMode(signal2redLED, OUTPUT);
  pinMode(signal3greenLED, OUTPUT);
  pinMode(signal3redLED, OUTPUT);
  pinMode(signal4greenLED, OUTPUT);
  pinMode(signal4redLED, OUTPUT);
  setSignalState(1, 2);
  setSignalState(2, 2);
  setSignalState(3, 2);
  setSignalState(4, 2);

  Serial.begin(115200);
  setup_wifi();
  client.setServer(MQTT_BROKER, 1883);
  client.setCallback(callback);
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(SSID);
 
    WiFi.begin(SSID, PSK);
 
    while (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(400);
      digitalWrite(LED_BUILTIN, LOW);
      delay(400);
      Serial.print(".");
    }
 
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
 
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length+1];
  for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      msg[i] = (char)payload[i];
  }
  Serial.println();

  msg[length] = '\0';
  // Serial.println(msg);

  // TODO: add flashing logic
  if(strcmp(msg, "T|Light 1|r")==0 || strcmp(msg, "T|Light 1|rf")==0){
    // signal red
    setSignalState(1, 2);
  }
  else if(strcmp(msg, "T|Light 1|g")==0 || strcmp(msg, "T|Light 1|gf")==0){
    // signal green
    setSignalState(1, 3);
  }
  else if(strcmp(msg, "T|Light 1|o")==0){
    // switch signal off
    setSignalState(1, 1);
  }

  if(strcmp(msg, "T|Light 2|r")==0 || strcmp(msg, "T|Light 2|rf")==0){
    // signal red
    setSignalState(2, 2);
  }
  else if(strcmp(msg, "T|Light 2|g")==0 || strcmp(msg, "T|Light 2|gf")==0){
    // signal green
    setSignalState(2, 3);
  }
  else if(strcmp(msg, "T|Light 2|o")==0){
    // switch signal off
    setSignalState(2, 1);
  }

  if(strcmp(msg, "T|Light 4|r")==0 || strcmp(msg, "T|Light 4|rf")==0){
    // signal red
    setSignalState(3, 2);
  }
  else if(strcmp(msg, "T|Light 4|g")==0 || strcmp(msg, "T|Light 4|gf")==0){
    // signal green
    setSignalState(3, 3);
  }
  else if(strcmp(msg, "T|Light 4|o")==0){
    // switch signal off
    setSignalState(3, 1);
  }

  if(strcmp(msg, "T|Light 5|r")==0 || strcmp(msg, "T|Light 5|rf")==0){
    // signal red
    setSignalState(4, 2);
  }
  else if(strcmp(msg, "T|Light 5|g")==0 || strcmp(msg, "T|Light 5|gf")==0){
    // signal green
    setSignalState(4, 3);
  }
  else if(strcmp(msg, "T|Light 5|o")==0){
    // switch signal off
    setSignalState(4, 1);
  }
}

void reconnect() {
    while (!client.connected()) {
        Serial.println("Reconnecting MQTT...");
        if (!client.connect("MattzoController Lights 1")) {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
    client.subscribe("nControl/websocket/rx");
    Serial.println("MQTT Connected.");
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

void setSignalState(int signalId, int state) {
// nControl signal states:
// 1: off
// 2: red
// 3: green
// 4: red flashing
// 5: green flashing

  switch(signalId) {
    case 1:
      if (state == 2 || state == 4) {
        Serial.println("Light 1 red");
        digitalWrite(signal1greenLED, HIGH);
        digitalWrite(signal1redLED, LOW);
      } else if (state == 3 || state == 5) {
        Serial.println("Light 1 green");
        digitalWrite(signal1greenLED, LOW);
        digitalWrite(signal1redLED, HIGH);
      } else {
        Serial.println("Light 1 off");
        digitalWrite(signal1greenLED, HIGH);
        digitalWrite(signal1redLED, HIGH);
      }
      break;
    case 2:
      if (state == 2 || state == 4) {
        Serial.println("Light 2 red");
        digitalWrite(signal2greenLED, HIGH);
        digitalWrite(signal2redLED, LOW);
      } else if (state == 3 || state == 5) {
        Serial.println("Light 2 green");
        digitalWrite(signal2greenLED, LOW);
        digitalWrite(signal2redLED, HIGH);
      } else {
        Serial.println("Light 2 off");
        digitalWrite(signal2greenLED, HIGH);
        digitalWrite(signal2redLED, HIGH);
      }
      break;
    case 3:
      if (state == 2 || state == 4) {
        Serial.println("Light 3 red");
        digitalWrite(signal3greenLED, HIGH);
        digitalWrite(signal3redLED, LOW);
      } else if (state == 3 || state == 5) {
        Serial.println("Light 3 green");
        digitalWrite(signal3greenLED, LOW);
        digitalWrite(signal3redLED, HIGH);
      } else {
        Serial.println("Light 3 off");
        digitalWrite(signal3greenLED, HIGH);
        digitalWrite(signal3redLED, HIGH);
      }
      break;
    case 4:
      if (state == 2 || state == 4) {
        Serial.println("Light 4 red");
        digitalWrite(signal4greenLED, HIGH);
        digitalWrite(signal4redLED, LOW);
      } else if (state == 3 || state == 5) {
        Serial.println("Light 4 green");
        digitalWrite(signal4greenLED, LOW);
        digitalWrite(signal4redLED, HIGH);
      } else {
        Serial.println("Light 4 off");
        digitalWrite(signal4greenLED, HIGH);
        digitalWrite(signal4redLED, HIGH);
      }
      break;
  }
}
