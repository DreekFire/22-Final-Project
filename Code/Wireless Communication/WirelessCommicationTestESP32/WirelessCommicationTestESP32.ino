#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char *SSID = "The Society of Indian Engineers";
const char *PWD = "BerkeleyInternationals2023";



char data[10];
int temp = 0;

// MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 

char *mqttServer = "192.168.4.168";
int mqttPort = 1883;


void connectToWiFi() {
  Serial.print("connecting to ");
 
  WiFi.begin(SSID, PWD);
  Serial.println(SSID);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.print("Connected.");
  
} 

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Callback - ");
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  // set the callback function
  mqttClient.setCallback(callback);
}


void setup() {
  Serial.begin(9600);
 
  connectToWiFi();
  setupMQTT();
    
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println("Connected.");
        // subscribe to topic
        mqttClient.subscribe("/swa/commands");
      }
      
  }
}


void loop() {

  if (!mqttClient.connected()) {
    reconnect();
  }

    mqttClient.loop();
    // Publishing data throgh MQTT

    temp = temp+1;
    char tempString[8];
    dtostrf(temp, 1, 2, tempString);
    mqttClient.publish("/swa/temperature", tempString);
    Serial.println(tempString);
    delay(500);
    if (temp == 100) {
      temp = -350;
      dtostrf(temp, 1, 2, tempString);
      mqttClient.publish("/swa/temperature", tempString);
      Serial.println(tempString);
      temp = 0;
    }



 
  }
