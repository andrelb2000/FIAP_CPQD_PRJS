#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <vector>
#include <algorithm>
#include "SensorData.h"
#include "DHT.h"

// Configurações por macro
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""

// Ajuste estes pinos conforme seu hardware (ESP32 usados como exemplo)
#define TEMP_PIN 15
#define GAS_PIN 35

#define sample_size 20  // Número de amostras para média móvel

// Labels e endpoint por macro
#define SENSOR_LABEL "Device02"
#define REST_ENDPOINT1 "http://host.wokwi.internal:9000/data"
#define REST_ENDPOINT2 "http://host.wokwi.internal:9001/data"

// Chaves JSON usadas no POST
#define JSON_KEY_LABEL "label"
#define JSON_KEY_AVG_TEMP "temp"
#define JSON_KEY_AVG_GAS "gas"

static bool useFirstEndpoint = true;

// Cria o objeto SensorData globalmente para armazenar leituras contínuas
SensorData sensor(SENSOR_LABEL);
DHT dht(TEMP_PIN, DHT22);

float minGasLevel = 4095.0; // Valor máximo para ADC 10 bits
float maxGasLevel = 0.0;    // Valor mínimo para ADC 10 bits

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
  dht.begin();
  Serial.println("DHT Inicializado!");
}

// Função responsável por fazer a leitura dos sensores e armazenar no objeto `sensor`.
void readSensors() {
  float rawTemp = dht.readTemperature();
  if(isnan(rawTemp)) {
    Serial.println("Falha ao ler do sensor DHT!");
    rawTemp = 0.0;
  }
  analogReadResolution(12);
  float rawGas = (float) analogRead(GAS_PIN);
  if(rawGas < minGasLevel) minGasLevel = rawGas;
  if(rawGas > maxGasLevel) maxGasLevel = rawGas;
  float calibratedGas = (rawGas - minGasLevel) / (maxGasLevel - minGasLevel) * 100.0;
  // evita valores nan no objeto sensor
  if(isnan(calibratedGas)) {
    calibratedGas = 0.0;
  }
  sensor.add(rawTemp, calibratedGas);
  // Exibe última leitura e médias
  Serial.print("Raw Temp: "); Serial.println(rawTemp); 
  Serial.print("Gas(raw): "); Serial.println(rawGas);
  Serial.print("Gas(cal): "); Serial.println(calibratedGas);
  Serial.print("Média Temp: "); Serial.println(sensor.avgTemp()); 
  Serial.print("Média Gas: "); Serial.println(sensor.avgGas());
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
  const char* REST_ENDPOINT = useFirstEndpoint ? REST_ENDPOINT1 : REST_ENDPOINT2;

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
     // Alterna servidor 
      useFirstEndpoint = !useFirstEndpoint;
  }
  http.end();
}
int sample_count = 0;
void loop() {
  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    readSensors();
    Serial.println("Conectado no WiFi há 1s");
    Serial.print("IP Local: ");
    Serial.println(WiFi.localIP());
    Serial.print("GateWay : ");
    Serial.println(WiFi.gatewayIP());
    // Envia médias para REST service
    Serial.println("-----------------------------------");
    Serial.println("Servidor REST: " + String(useFirstEndpoint ? REST_ENDPOINT1 : REST_ENDPOINT2));
    Serial.println("-----------------------------------");
    sendSensorAverages();
    if(++sample_count >= sample_size) {
      sample_count = 0;
      sensor.clear();
      Serial.println("Dados do sensor limpos para nova coleta."); 
    }    
  }
}