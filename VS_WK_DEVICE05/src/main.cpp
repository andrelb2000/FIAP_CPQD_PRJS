#include <Arduino.h>
#include "esp_spi_flash.h"
#include "mbedtls/sha256.h"

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

#define VIOLATION_PIN 4 // pino do push-button que indica violação física
#define FIRM_VIOLATION_PIN 18 // pino do push-button que indica violação de firmware

#define sample_size 20  // Número de amostras para média móvel

// Labels e endpoint por macro
#define SENSOR_LABEL "Device05"
#define REST_ENDPOINT1 "http://host.wokwi.internal:9000/data"
#define REST_ENDPOINT2 "http://host.wokwi.internal:9001/data"

// Chaves JSON usadas no POST
#define JSON_KEY_LABEL "label"
#define JSON_KEY_AVG_TEMP "temp"
#define JSON_KEY_AVG_GAS "gas"
#define JSON_KEY_SECURITY "security"


static bool useFirstEndpoint = true;


#define APP_OFFSET 0x10000    // Offset padrão do app
#define APP_SIZE   0x100000   // 1 MB (suficiente para demo)

// Hash SHA-256 esperado do firmware (atualize conforme necessário)
// Rode a primeira vez para obter o hash correto e substitua aqui - Este Hash seria obtido externamente em um sistema real.
const char expected_hash[] = "cf1f831c3bd0674082a9a798149b5c4a754907a17736160b09cf4a14a0c87dd8";

// Valor mutavel do hash esperado — pode ser atualizado pela leitura do botão
static String current_expected_hash = String(expected_hash);

// Somente um recurso para garantir um HASH fixo para o manifesto do firmware
static const char FIRMWARE_MANIFEST[] =
  "APP=DEMO_SECUREBOOT_FIAP;"
  "DEVICE=ESP32;"
  "VER=1.0.0;"
  "BUILD_PROFILE=CLASSROOM;"
  "SALT=9f2c1a77";  // pode ser qualquer coisa fixa

// Cria o objeto SensorData globalmente para armazenar leituras contínuas
SensorData sensor(SENSOR_LABEL);
DHT dht(TEMP_PIN, DHT22);

// Variável booleana indicando violação física do dispositivo
static bool violacaoFisica = false;

// Variável booleana indicando violação física do dispositivo
static bool violacaoFirmware = false;

// Variável enviada pelo serviço REST com valor "SEGURO" ou "VIOLADO"
static String securityStatus = "SEGURO";

float minGasLevel = 4095.0; // Valor máximo para ADC 12 bits
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

// Converte bytes -> hex
static void toHex(const uint8_t in[32], char out[65]) {
  for (int i = 0; i < 32; i++) sprintf(&out[i * 2], "%02x", in[i]);
  out[64] = '\0';
}

// Calcula SHA-256 de um buffer
static void sha256_of_string(const char* s, uint8_t out[32]) {
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);                 // 0 = SHA-256
  mbedtls_sha256_update(&ctx, (const unsigned char*)s, strlen(s));
  mbedtls_sha256_finish(&ctx, out);
  mbedtls_sha256_free(&ctx);
}

// Verifica a integridade do firmware lendo a memória flash e calculando o hash SHA-256
bool verifyFirmwareIntegrity() {
  uint8_t hash[32];
  char hash_hex[65];

  // Calcula hash do manifesto
  sha256_of_string(FIRMWARE_MANIFEST, hash);
  toHex(hash, hash_hex);

  Serial.println("=== Secure Boot SIMULADO (Wokwi) ===");
  Serial.print("Manifest: ");
  Serial.println(FIRMWARE_MANIFEST);
  Serial.print("Hash(manifest): ");
  Serial.println(hash_hex);

  // Se EXPECTED_HASH vazio, só mostramos o hash para você copiar.
  if (strlen(expected_hash) == 0) {
    Serial.println("\expected_hash está vazio.");
    Serial.println("Copie o Hash(manifest) acima e cole em EXPECTED_HASH.");
    Serial.println("✗ FALHA DE INTEGRIDADE (firmware NÃO autorizado)");
    return false; 
  }

  if (strcmp(hash_hex, current_expected_hash.c_str()) == 0) {
    Serial.println("✓ Integridade OK (firmware autorizado)");
    return true;
  } else {
    Serial.println("✗ FALHA DE INTEGRIDADE (firmware NÃO autorizado)");
    return false;
  }
}

// Lê um pino digital `readings` vezes, monta a sequência de '0'/'1', calcula SHA-256
// e substitui `current_expected_hash` pelo novo valor gerado (hex).
void readFirmViolation(uint8_t pin, int readings, int delayMs) {
  if (readings <= 0) return;
  // Primeiro checa um pulso rápido para decidir iniciar a amostragem.
  int v0 = digitalRead(pin);
  int bit0 = (v0 == LOW) ? 1 : 0; // INPUT_PULLUP: pressed == LOW
  if (bit0 == 0) {
    // primeiro valor não indica início (não é '1') -> ignora
    Serial.println("[readFirmViolation] primeiro bit = 0; amostragem ignorada.");
    return;
  }
  // Se chegou aqui, o primeiro bit foi 1 -> realiza amostragem completa
  String seq;
  seq.reserve(readings);
  int ones = 0;
  seq += '1';
  ones = 1;
  // já consumimos a primeira leitura; agora coleta as demais
  for (int i = 1; i < readings; ++i) {
    delay(delayMs);
    int v = digitalRead(pin);
    int bit = (v == LOW) ? 1 : 0;
    seq += (bit ? '1' : '0');
    if (bit) ones++;
  }
  // calcula sha256 da sequência
  uint8_t h[32];
  sha256_of_string(seq.c_str(), h);
  char hex[65];
  toHex(h, hex);
  current_expected_hash = String(hex);
  Serial.print("[readFirmViolation] seq: "); Serial.println(seq);
  Serial.print("[readFirmViolation] novo hash: "); Serial.println(current_expected_hash);
  double avg = (double)ones / (double)readings;
  if (avg > 0.2 && avg < 0.8) {
    Serial.println("[readFirmViolation] sinal instavel (possivel ruido/hardware)");
  }
}


// Lê o pino digital do push-button e atualiza variáveis de violação
void TestarViolacao() {
  // Usa INPUT_PULLUP: botão pressionado conecta ao GND -> LOW
  int val = digitalRead(VIOLATION_PIN);
  if (val == LOW) {
    violacaoFisica = true;
    securityStatus = "VIOLADO";
  }
  Serial.print("Violacao fisica: "); Serial.println(violacaoFisica ? "SIM" : "NAO");
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
  // adiciona status de segurança (SEGURO / VIOLADO)
  doc[JSON_KEY_SECURITY] = securityStatus;

  String payload;
  serializeJson(doc, payload);
  Serial.print("Payload JSON: "); Serial.println(payload);
  int httpCode = http.POST(payload);
  if (httpCode > 0) {
    Serial.print("POST código: "); Serial.println(httpCode);
    String resp = http.getString();
    Serial.print("Resposta: "); Serial.println(resp);
  } else {
    Serial.print("Erro envio HTTP: "); Serial.println(httpCode);
    useFirstEndpoint = !useFirstEndpoint;
  }
  http.end();
}
int sample_count = 0;


unsigned long lastMsg = 0;
void setup() {
  Serial.begin(115200);
  connectWiFi();
  dht.begin();
  Serial.println("DHT Inicializado!");
  // Configura o pino do botão de violação como entrada com pull-up
  pinMode(VIOLATION_PIN, INPUT_PULLUP);
  pinMode(FIRM_VIOLATION_PIN, INPUT_PULLUP);
  if (!verifyFirmwareIntegrity()) {
    violacaoFirmware = true;
    securityStatus = "FIRMWARE VIOLADO";
    Serial.println("Sistema bloqueado.");
    sendSensorAverages();
    while (true) {
      delay(1000);
    }
  }
}
void loop() {
  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    // Atualiza estado de violação física a cada ciclo
    TestarViolacao();
    readFirmViolation(FIRM_VIOLATION_PIN, sample_size, 20);
    if (violacaoFisica || violacaoFirmware) {
      // Em caso de violação, apenas reporta no monitor serial e não faz mais nada
      Serial.println("!!!! VIOLACAO FISICA/Firmware DETECTADA: envio de dados suspenso !!!!");
      sendSensorAverages();
      while(true) {
        delay(1000);
      }
    } else {
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
      if (!verifyFirmwareIntegrity()) {
        violacaoFirmware = true;
        securityStatus = "FIRMWARE VIOLADO";
        Serial.println("Sistema sera bloqueado.");        
      } 
      sendSensorAverages();
      if(++sample_count >= sample_size) {
        sample_count = 0;
        sensor.clear();
        Serial.println("Dados do sensor limpos para nova coleta."); 
      }

    }
  }
}