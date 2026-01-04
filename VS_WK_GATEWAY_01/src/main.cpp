#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

WebServer server(80);

bool hasData = false;

String  lastText = "";
long    lastInteger = 0;
double  lastDecimal = 0.0;

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

void handlePostData() {
  Serial.println("\n Entrei no POST/data");
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  // Prefer JSON body (application/json). Fallback to form-urlencoded.
  String body = server.arg("plain");
  if (body.length() > 0 && body.charAt(0) == '{') {
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      server.send(400, "application/json", "{\"error\":\"invalid json\"}");
      server.client().stop();
      return;
    }
    if (!doc.containsKey("text") || !doc.containsKey("integer") || !doc.containsKey("decimal")) {
      server.send(400, "application/json", "{\"error\":\"missing fields (text, integer, decimal)\"}");
      server.client().stop();
      return;
    }
    lastText = String((const char*) doc["text"].as<const char*>());
    lastInteger = doc["integer"].as<long>();
    lastDecimal = doc["decimal"].as<double>();
    hasData = true;
    server.send(200, "application/json", "{\"status\":\"ok\"}");
    server.client().stop();
    return;
  }

  // Fallback: form-urlencoded
  String text = server.arg("text");
  String sInt = server.arg("integer");
  String sDec = server.arg("decimal");

  if (text.length() == 0 || sInt.length() == 0 || sDec.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"missing fields (text, integer, decimal)\"}");
    server.client().stop();
    return;
  }

  lastText = text;
  lastInteger = sInt.toInt();
  lastDecimal = sDec.toFloat();
  hasData = true;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
  server.client().stop();
}

void handleGetData() {
  Serial.println("\n Entrei no GET/data");
  if (!hasData) {
    server.send(204, "text/plain", "");
    Serial.println("\n Sem dados pra retornar no GET/data");
    return;
  }
  String out = "{";
  out += "\"text\":\"" + escapeJson(lastText) + "\",";
  out += "\"integer\":" + String(lastInteger) + ",";
  out += "\"decimal\":" + String(lastDecimal, 6);
  out += "}";
  server.send(200, "application/json", out);
}

void handleStatus() {
  String out = "{";
  out += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  out += "\"connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  out += "}";
  server.send(200, "application/json", out);
}

void setup() {
  Serial.begin(115200);
  connectWiFi();

  server.on("/data", HTTP_POST, handlePostData);
  server.on("/data", HTTP_GET, handleGetData);
  server.on("/status", HTTP_GET, handleStatus);

  server.begin();
  Serial.println("REST server iniciado em /data (POST/GET)");
}

void loop() {
  server.handleClient();
}