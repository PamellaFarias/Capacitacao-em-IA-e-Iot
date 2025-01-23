#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* mqttServer = "200.129.71.138";
const int mqttPort = 1883;
const char* mqttUser = "ayrtton_silva:ff3908";
const char* mqttTopic_PUBLISH_PIR = "ayrtton_silva:ff3908/attrs";
const char* mqttTopic_SUBSCRIBE_RELAY = "ayrtton_silva:ff3908/config";

WiFiClient espClient;
PubSubClient client(espClient);

const int PIR_PIN = 26;
const int RELAY_PIN = 27;

bool flag = 0;
int pirState = 0;
unsigned long start_time;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando ao broker MQTT... ");
    if (client.connect("ESP32Client", mqttUser, "")) {
      Serial.println("conectado");
      client.subscribe(mqttTopic_SUBSCRIBE_RELAY);
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(". Tentando novamente em 5 segundos.");
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String msg;

  for (int i = 0; i < length; i++) {
    char c = (char)payload[i];

    msg += c;
  }

  Serial.printf("Chegou a seguinte string via MQTT: %s do topico: %s\n", msg, topic);

  if (msg.equals("1")) {
    digitalWrite(RELAY_PIN, HIGH);
    Serial.print("Relé ligado por comando MQTT");
  }
  if (msg.equals("0")) {
    digitalWrite(RELAY_PIN, LOW);
    Serial.print("Relé desligado por comando MQTT");
  }
}

void publish_PIR(int pirState){
  char payload[32];
  sprintf(payload, "{\"pirState\": %d}", pirState);

  while (!client.publish(mqttTopic_PUBLISH_PIR, payload)) {
    Serial.println("Falha ao publicar a mensagem. Tentando novamente...");
    delay(500);
  }
  Serial.println("Mensagem publicada.");
  
  memset(payload, 0, sizeof(payload));
}

void state_machine_task(void *pvParameter) {
  unsigned long lastDetectionTime = 0;
  bool isLedOn = false;

  while (1) {
    int pirReading = digitalRead(PIR_PIN);

    if (pirReading == HIGH) { 
      lastDetectionTime = millis();

      if (!isLedOn) {
        isLedOn = true;
        pirState = 1;
        publish_PIR(pirState);
        digitalWrite(RELAY_PIN, HIGH);
        Serial.println("Sensor ativado.");
      }
    }
    if (isLedOn && (millis() - lastDetectionTime > 5000)) {
      isLedOn = false;
      pirState = 0;
      publish_PIR(pirState);
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Sensor desativado.");
    }

    delay(10);
  }
}

void sensor_isr_handler() {
  pirState = 1;
}

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIR_PIN), sensor_isr_handler, RISING);

  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  xTaskCreate(&state_machine_task, "state_machine_task", 2048, NULL, 5, NULL);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
