#include <ESP8266WiFi.h>  // WiFi library
#include <PubSubClient.h>  // MQTT library
#include <Servo.h>  // servo library

const char* SSID = "TBW13";
const char* PSK = "tbw13iscool";
const char* MQTT_BROKER = "192.168.1.19";
const char* HOST = "MattzoController Switch 2/3/7";

const int led = D0;

WiFiClient espClient;
PubSubClient client(espClient);
// long lastMsg = 0;
// char msg[255];
// int value = 0;

// create servo objects to control servos
// Servo servo1;
Servo servo2;
Servo servo3;
// Servo servo4;
// Servo servo5;
// Servo servo6;
Servo servo7;
// Servo servo8;

// TrixBrix Lego-Switches
// const int servMin1 = 72;
// const int servMax1 = 86;

// TrixBrix left / right switches
//const int servMin1 = 60;
//const int servMax1 = 93;

// Switch 2
const int servMin2 = 66;  // turn
const int servMax2 = 90;  // straight

// Switch 3
const int servMin3 = 85;  // turn
const int servMax3 = 60;  // straight

// Switch 7
const int servMin7 = 91;  // turn
const int servMax7 = 65;  // straight

//const int servMin4 = 56;
//const int servMax4 = 92;
//const int servMin5 = 53;
//const int servMax5 = 84;
//const int servMin6 = 58;
//const int servMax6 = 85;
//const int servMin8 = 60;
//const int servMax8 = 90;

// delay between to switch operations
const int switchDelay = 200;

void setup() {
  // pinMode(operationLED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led, OUTPUT);

//  servo1.attach(D1);
  servo2.attach(D2);
  servo3.attach(D3);
//  servo4.attach(D4);
//  servo5.attach(D5);
//  servo6.attach(D6);
  servo7.attach(D7);
//  servo8.attach(D8);

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
      digitalWrite(led, LOW);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(led, HIGH);
      delay(500);
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

//  if(strcmp(msg, "T|Switch 1|s")==0){
//    Serial.println("Switch 1 straight");
//    servo1.write(servMax1);
//    delay(switchDelay);
//  }
//  else if(strcmp(msg, "T|Switch 1|t")==0){
//    Serial.println("Switch 1 turn");
//    servo1.write(servMin1);
//    delay(switchDelay);
//  }

  if(strcmp(msg, "T|Switch 2|s")==0){
    Serial.println("Switch 2 straight");
    servo2.write(servMax2);
    delay(switchDelay);
  }
  else if(strcmp(msg, "T|Switch 2|t")==0){
    Serial.println("Switch 2 turn");
    servo2.write(servMin2);
    delay(switchDelay);
  }

  if(strcmp(msg, "T|Switch 3|s")==0){
    Serial.println("Switch 3 straight");
    servo3.write(servMax3);
    delay(switchDelay);
  }
  else if(strcmp(msg, "T|Switch 3|t")==0){
    Serial.println("Switch 3 turn");
    servo3.write(servMin3);
    delay(switchDelay);
  }

//  if(strcmp(msg, "T|Switch 4|s")==0){
//    Serial.println("Switch 4 straight");
//    servo4.write(servMax4);
//    delay(switchDelay);
//  }
//  else if(strcmp(msg, "T|Switch 4|t")==0){
//    Serial.println("Switch 4 turn");
//    servo4.write(servMin4);
//    delay(switchDelay);
//  }
//
//  if(strcmp(msg, "T|Switch 5|s")==0){
//    Serial.println("Switch 5 straight");
//    servo5.write(servMax5);
//    delay(switchDelay);
//  }
//  else if(strcmp(msg, "T|Switch 5|t")==0){
//    Serial.println("Switch 5 turn");
//    servo5.write(servMin5);
//    delay(switchDelay);
//  }
//
//  if(strcmp(msg, "T|Switch 6|s")==0){
//    Serial.println("Switch 6 straight");
//    servo6.write(servMax6);
//    delay(switchDelay);
//  }
//  else if(strcmp(msg, "T|Switch 6|t")==0){
//    Serial.println("Switch 6 turn");
//    servo6.write(servMin6);
//    delay(switchDelay);
//  }

  if(strcmp(msg, "T|Switch 7|s")==0){
    Serial.println("Switch 7 straight");
    servo7.write(servMax7);
    delay(switchDelay);
  }
  else if(strcmp(msg, "T|Switch 7|t")==0){
    Serial.println("Switch 7 turn");
    servo7.write(servMin7);
    delay(switchDelay);
  }

//  if(strcmp(msg, "T|Switch 8|s")==0){
//    Serial.println("Switch 8 straight");
//    servo8.write(servMax8);
//    delay(switchDelay);
//  }
//  else if(strcmp(msg, "T|Switch 8|t")==0){
//    Serial.println("Switch 8 turn");
//    servo8.write(servMin8);
//    delay(switchDelay);
//  }
}

void reconnect() {
    while (!client.connected()) {
        Serial.println("Reconnecting MQTT...");
        if (!client.connect(HOST)) {
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" retrying in 5 seconds");
          digitalWrite(LED_BUILTIN, LOW);
          digitalWrite(led, HIGH);
          delay(250);
          digitalWrite(LED_BUILTIN, HIGH);
          digitalWrite(led, LOW);
          delay(1500);
          digitalWrite(LED_BUILTIN, LOW);
          digitalWrite(led, HIGH);
          delay(250);
          digitalWrite(LED_BUILTIN, HIGH);
          digitalWrite(led, LOW);
          delay(1500);
          digitalWrite(LED_BUILTIN, LOW);
          digitalWrite(led, HIGH);
          delay(250);
          digitalWrite(LED_BUILTIN, HIGH);
          digitalWrite(led, LOW);
        }
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(led, HIGH);
    client.subscribe("nControl/websocket/rx");
    Serial.println("MQTT Connected.");
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}
