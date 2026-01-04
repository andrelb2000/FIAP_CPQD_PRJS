#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <vector>
#include <algorithm>
#include "SensorData.h"

// Configurações por macro
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""

// Ajuste estes pinos conforme seu hardware (ESP32 usados como exemplo)
#define TEMP_PIN 34
#define GAS_PIN 35

// Labels e endpoint por macro
#define SENSOR_LABEL "Device01"
#define REST_ENDPOINT "http://host.wokwi.internal:9000/data"

// Chaves JSON usadas no POST
#define JSON_KEY_LABEL "label"
#define JSON_KEY_AVG_TEMP "temp"
#define JSON_KEY_AVG_GAS "gas"

// Cria o objeto SensorData globalmente para armazenar leituras contínuas
SensorData sensor(SENSOR_LABEL);

void connectWiFi() {
  Serial.print("[WiFi] Conectando...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Conectado!");
}

unsigned long lastMsg = 0;
void setup() {
  Serial.begin(115200);
  connectWiFi();
}

// Função responsável por fazer a leitura dos sensores e armazenar no objeto `sensor`.
void readSensors() {
  int rawTemp = analogRead(TEMP_PIN);
  int rawGas = analogRead(GAS_PIN);

  // Conversão simples para temperatura (exemplo para sensor tipo TMP36/LM35 no ESP32)
  double voltage = rawTemp * (3.3 / 4095.0);
  double temperatureC = voltage * 100.0; // ajuste conforme sensor

  sensor.add(temperatureC, rawGas);

  // Exibe última leitura e médias
  Serial.print("Leitura - Temp: "); Serial.print(temperatureC); Serial.print(" C, Gas(raw): "); Serial.println(rawGas);
  Serial.print("Média Temp: "); Serial.print(sensor.avgTemp()); Serial.print(" C, Média Gas: "); Serial.println(sensor.avgGas());
}

// Envia via HTTP POST o rótulo e as médias atuais do objeto `sensor`.
void sendSensorAverages() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi não conectado; tentando reconectar (até 10 tentativas)...");
    const int maxAttempts = 10;
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
      attempts++;
      Serial.print("Tentativa de reconexão #"); Serial.println(attempts);
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      unsigned long start = millis();
      // aguarda até 2s por tentativa, checando periodicamente
      while (millis() - start < 2000 && WiFi.status() != WL_CONNECTED) {
        delay(200);
      }
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Falha ao reconectar após 10 tentativas; cancelando envio REST");
      return;
    }
    Serial.println("Reconectado ao WiFi.");
  }
  HTTPClient http;
  http.begin(REST_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  StaticJsonDocument<256> doc;
  doc[JSON_KEY_LABEL] = sensor.label;
  doc[JSON_KEY_AVG_TEMP] = sensor.avgTemp();
  doc[JSON_KEY_AVG_GAS] = sensor.avgGas();
  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);
  if (httpCode > 0) {
    Serial.print("POST código: "); Serial.println(httpCode);
    String resp = http.getString();
    Serial.print("Resposta: "); Serial.println(resp);
  } else {
    Serial.print("Erro envio HTTP: "); Serial.println(httpCode);
  }
  http.end();
}

void loop() {
  unsigned long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    readSensors();
    Serial.println("Conectado no WiFi há 1s");
    Serial.print("IP Local: ");
    Serial.println(WiFi.localIP());
    Serial.print("GateWay : ");
    Serial.println(WiFi.gatewayIP());
    // Envia médias para REST service
    sendSensorAverages();
  }
}