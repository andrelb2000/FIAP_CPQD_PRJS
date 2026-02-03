#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <vector>
#include <algorithm>
#include "DHT.h"
// Configurações por macro
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
// Ajuste estes pinos conforme seu hardware (ESP32 usados como exemplo)
#define TEMP_PIN 15
#define REST_THINGSPEAK "https://api.thingspeak.com/update"
String apiKey = "B06MJJSWGIAY2ITV";
DHT dht(TEMP_PIN, DHT22);

void connectWiFi() {
  Serial.print("[WiFi] Conectando...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Conectado!");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  connectWiFi();
}

void loop() {
  // put your main code here, to run repeatedly:
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("[DHT] Falha ao ler do sensor!");
    return;
  }
  Serial.print("[DHT] Temperatura: ");
  Serial.print(temperature);  Serial.print(" °C, Umidade: ");
  Serial.print(humidity);   Serial.println(" %");
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(REST_THINGSPEAK) + "?api_key=" + apiKey +
                 "&field1=" + String(temperature) +
                 "&field2=" + String(humidity);
    http.begin(url);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.print("[HTTP] Código de resposta: ");
      Serial.println(httpResponseCode);
      Serial.print("[HTTP] Resposta: ");
      Serial.println(response);
    } else {
      Serial.print("[HTTP] Erro na requisição: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("[WiFi] Desconectado!");
  }
  delay(18000); // Aguarda 60 segundos antes da próxima leitura 
}
