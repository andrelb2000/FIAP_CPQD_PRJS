#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <vector>
#include <algorithm>
#include "SensorData.h"

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

const char* MQTT_BROKER = "industrial.api.ubidots.com";
const int   MQTT_PORT   = 1883;
const char* MQTT_CLIENTID = "";

const char* UBIDOTS_TOKEN = "BBUS-MqjWH3tL051NmtmeNEZ38K8SCwPydT"; // Username no MQTT
const char* MQTT_SUB_TOPIC = "/v1.6/devices/fiap_ambiente/parada/lv";
const char* MQTT_PUB_TOPIC = "/v1.6/devices/fiap_ambiente";

char msg[255];
int   parada = 0;

WiFiClient espClient;
PubSubClient mqtt(espClient);
WebServer server(80);
SensorData agg = SensorData();

// Array global para guardar dados de múltiplos dispositivos
std::vector<SensorData> devices;
bool hasData = false;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Mensagem em [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  payload[length] = 0;
  if(strcmp( (const char *)payload,"1.0")==0){
    parada = 1;
  }else{
    parada = 0;
  }
  
  Serial.println();
}
void connectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Conectando...");
 //   if (mqtt.connect(MQTT_CLIENTID)) {

    if (mqtt.connect(MQTT_CLIENTID,UBIDOTS_TOKEN,"")) {
      Serial.println(" conectado!");
      mqtt.subscribe(MQTT_SUB_TOPIC);
    } else {
      Serial.print(" falhou (rc=");
      Serial.print(mqtt.state());
      Serial.println("). Tentando novamente em 2s...");
      delay(2000);
    }
  }
}
void connectWiFi() {
  Serial.print("[WiFi] Conectando...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

String escapeJson(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s.charAt(i);
    if (c == '"') o += "\\\"";
    else if (c == '\\') o += "\\\\";
    else o += c;
  }
  return o;
}

// Agrega uma lista de SensorData em um único SensorData
SensorData aggregateDevices(const std::vector<SensorData> &list) {
  // Resultado: um único SensorData contendo:
  // - `label`: concatenação dos rótulos de entrada, separados por vírgula
  // - `temps`: um único valor — a média de todas as temperaturas recebidas
  // - `gasLevels`: um único valor — o máximo observado entre todos os níveis de gás
  // Observações de comportamento:
  // - Se não houver temperaturas, `temps` ficará vazio e a média será 0.0 internamente.
  // - Se não houver valores de gás, `gasLevels` ficará vazio.
  // - Complexidade: O(n) em relação ao número total de leituras (varredura única).
  SensorData out("aggregate");

  // Acumuladores temporários
  double sum = 0.0;     // soma de todas as temperaturas para calcular a média
  size_t count = 0;     // número total de leituras de temperatura
  int maxGas = -1;      // maior valor de gás encontrado; -1 indica nenhum valor
  String labels = "";  // concatena rótulos encontrados

  // Percorre todos os dispositivos fornecidos
  for (const auto &d : list) {
    // Concatena rótulos não vazios, separando por vírgula
    if (d.label.length() > 0) {
      if (labels.length() == 0) labels = d.label;
      else labels += "," + d.label;
    }

    // Soma todas as temperaturas e conta quantas foram somadas
    for (double v : d.temps) { sum += v; ++count; }

    // Registra o maior nível de gás observado entre todos os dispositivos
    for (int g : d.gasLevels) {
        if ((g > maxGas)) 
           maxGas = g;
        Serial.print("Max gás atual: ");
        Serial.println(g);
    }

  }

  // Calcula média apenas se houver ao menos uma leitura
  double avg = 0.0;
  if (count > 0) avg = sum / (double)count;

  // Monta o objeto de saída: rótulo agregado e valores agregados (se existirem)
  out.label = labels;
  if (count > 0) out.temps.push_back(avg);    // média das temperaturas
  if (maxGas >= 0) out.gasLevels.push_back(maxGas); // máximo de gás
  return out;
}
void handlePostData() {
  Serial.println("\n Entrei no POST/data (sensor single values)");
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  // Aceita JSON no body ou form-urlencoded (label, temp, gas)
  String body = server.arg("plain");
  SensorData incoming;
  bool valid = false;

  if (body.length() > 0 && body.charAt(0) == '{') {
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      server.send(400, "application/json", "{\"error\":\"invalid json\"}");
      server.client().stop();
      return;
    }
    if (!doc.containsKey("label") || !doc.containsKey("temp") || !doc.containsKey("gas")) {
      server.send(400, "application/json", "{\"error\":\"missing fields (label, temp, gas)\"}");
      server.client().stop();
      return;
    }
    double tempVal = doc["temp"].as<double>();
    int gasVal = doc["gas"].as<int>();
    incoming.label = String((const char*)doc["label"].as<const char*>());
    incoming.add(tempVal, gasVal);
    valid = true;
  } else {
    // fallback form data
    String label = server.arg("label");
    String sTemp = server.arg("temp");
    String sGas = server.arg("gas");
    if (label.length() > 0 && sTemp.length() > 0 && sGas.length() > 0) {
      double tempVal = sTemp.toFloat();
      double gasVal = sGas.toFloat();
      incoming.label = label;
      incoming.add(tempVal, gasVal);
      valid = true;
    }
  }

  if (!valid) {
    server.send(400, "application/json", "{\"error\":\"missing fields (label, temp, gas)\"}");
    server.client().stop();
    return;
  }

  // Procurar dispositivo com mesmo label e acrescentar, caso contrário adicionar novo
  bool found = false;
  for (auto &d : devices) {
    if (d.label == incoming.label) {
      // Adiciona apenas um novo valor por requisição
      if (!incoming.temps.empty() && !incoming.gasLevels.empty()) {
        d.temps[0] = incoming.temps[0];
        d.gasLevels[0] = incoming.gasLevels[0];
      }
      found = true;
      break;
    }
  }
  if (!found) devices.push_back(incoming);

  hasData = !devices.empty();
  Serial.println("\n Dados recebidos e armazenados no POST/data");
  Serial.print(" Total dispositivos armazenados: ");
  Serial.println(devices.size());
  Serial.print(" Dispositivo atual: ");
  Serial.println(incoming.label);
  Serial.print(" Temperatura recebida: ");
  Serial.println(incoming.temps[0]);
  Serial.print(" Nível de gás recebido: ");
  Serial.println(incoming.gasLevels[0]);
  
  agg = aggregateDevices(devices);

  server.send(200, "application/json", "{\"status\":\"ok\"}");
  server.client().stop();
}
void handleGetData() {
  Serial.println("\n Entrei no GET/data (aggregate)");
  if (!hasData) {
    server.send(204, "text/plain", "");
    Serial.println("\n Sem dados pra retornar no GET/data");
    return;
  }

  SensorData agg = aggregateDevices(devices);
  DynamicJsonDocument doc(256);
  doc["label"] = agg.label;
  JsonArray t = doc.createNestedArray("temp");
  for (double v : agg.temps) t.add(v);
  JsonArray g = doc.createNestedArray("gas");
  for (int v : agg.gasLevels) g.add(v);

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}
void handleStatus() {
  String out = "{";
  out += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  out += "\"connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  out += "}";
  server.send(200, "application/json", out);
}

unsigned long lastMsg = 0; 
void setup() {
  Serial.begin(115200);
  connectWiFi();
  server.on("/data", HTTP_POST, handlePostData);
  server.on("/data", HTTP_GET, handleGetData);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
  Serial.println("REST server iniciado em /data (POST/GET)");

  connectMQTT();
}

void loop() {
  if (!mqtt.connected()) 
     connectMQTT();
  mqtt.loop();

  unsigned long now = millis();
  if(parada){
     Serial.println("Sistema parado ");
  }else{
    if (now - lastMsg > 500) {
      lastMsg = now;
      server.handleClient();
      sprintf(msg,"{\"temperatura\": %3.2lf, \"gas\": %5.1lf, \"parada\": %1i}",
                  agg.avgTemp(),agg.avgGas(),parada); 
      if(devices.size()>0){
        Serial.print("Dispositivos agregados: ");
        Serial.println(devices.size());
        mqtt.publish(MQTT_PUB_TOPIC, msg);
        Serial.print("[MQTT] Publicado: ");
        Serial.println(msg);
      }
    }
  }
}