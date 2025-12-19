/*
  ESP32 - Publica temperatura por MQTT + publica evento de sensor de proximidad
  (Solicita la IP/dominio del broker por Serial al iniciar)

  - Lee temperatura y publica la temperatura de forma periódica
  - Lee una entrada digital para sensor de proximidad y publica true/false según detección
  - Pide por Serial la dirección (IP o dominio) del broker MQTT al arrancar
  - Usa WiFi + ESP32MQTTClient
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// ========== CONFIG ========
const char* ssid        = "DESKTOP-RTTQ50L 0411";
const char* wifiPass    = "574;Cm23";

char mqtt_server[50] = "";  // se pedirá por Serial al inicio
const uint16_t mqtt_port = 1883;

const char* tempTopic   = "esp32/temperature";
const char* proxTopic   = "esp32/proximity";

const unsigned long TEMP_PUBLISH_INTERVAL = 1000UL; // publicar temp cada 1s
const unsigned long PROX_DEBOUNCE_MS = 200; // debounce de proximidad en ms
// ==========================

// Pines
const uint8_t TEMP_PIN = 34;    // Pin entrada digital del sensor de temperatura
const uint8_t PROX_PIN = 15;    // Pin entrada digital del sensor de proximidad

ESP32MQTTClient mqttClient;

DHT_Unified dht(TEMP_PIN, DHT11);

unsigned long lastTempPublish = 0;
unsigned long lastProxChange = 0;

// Variables para el sensor de proximidad
bool lastProxState = false;
bool currentProxState = false;
bool proxTriggered = false;

// ---------- Helpers ----------
void setupWiFi() {
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, wifiPass);
  
  pinMode(PROX_PIN,INPUT);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nTimeout WiFi. Reintentando...");
      start = millis();
      WiFi.begin(ssid, wifiPass);
    }
  }
  
  Serial.println();
  Serial.print("WiFi conectado. IP: ");
  Serial.println(WiFi.localIP());
}

float readTemperature() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  float Temperature = event.temperature;
  return Temperature;
}

void publishTemperature() {
  float tempC = readTemperature();
  
  char payload[32];
  snprintf(payload, sizeof(payload), "%.2f", tempC);
  
  bool ok = mqttClient.publish(tempTopic, payload, 0, false);
  
  Serial.print("Publicado temperatura: ");
  Serial.print(payload);
  Serial.print("°C -> ");
  Serial.println(ok ? "OK" : "FALLA");
}

void publishProximityState(bool detected) {
  // Publica "true" si hay objeto, "false" si no hay
  const char* payload = detected ? "true" : "false";
  
  bool ok = mqttClient.publish(proxTopic, payload, 0, false);
  
  Serial.print("Publicado proximidad: ");
  Serial.print(payload);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FALLA");
}

// Función para leer sensor de proximidad con debounce
bool readProximitySensor() {
  // Leer el pin - para sensor NPN normalmente abierto: LOW = detectado
  bool rawState = (digitalRead(PROX_PIN) == HIGH); // TRUE cuando detecta objeto
  
  unsigned long now = millis();
  
  // Debounce simple
  if (rawState != currentProxState) {
    if (now - lastProxChange > PROX_DEBOUNCE_MS) {
      currentProxState = rawState;
      lastProxChange = now;
      proxTriggered = true;
    }
  }
  
  return currentProxState;
}

// Required global callback for connection events
void onMqttConnect(esp_mqtt_client_handle_t client) {
  if (mqttClient.isMyTurn(client)) {
    Serial.println("MQTT conectado!");
  }
}

// Required global event handler
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
esp_err_t handleMQTT(esp_mqtt_event_handle_t event) {
  mqttClient.onEventCallback(event);
  return ESP_OK;
}
#else
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  mqttClient.onEventCallback(event);
}
#endif

// Lee una línea desde Serial
String readSerialLineBlocking(const char* prompt) {
  Serial.println(prompt);
  String s = "";
  unsigned long timeout = millis() + 30000; // 30 segundos timeout
  
  while (millis() < timeout) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (s.length() > 0) {
          s.trim();
          break;
        }
      } else {
        s += c;
      }
    }
    delay(10);
  }
  
  if (s.length() == 0) {
    s = "192.168.1.100"; // IP por defecto
    Serial.println("Usando IP por defecto: " + s);
  }
  
  return s;
}

// ========== Setup y loop ==========
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n\n=== Sistema MQTT Temperatura/Proximidad ===");
  
  // Pedir al usuario por Serial la dirección del broker MQTT
  Serial.println("\n== Configuración MQTT ==");
  Serial.println("Introduce la IP del broker MQTT (ej: 192.168.1.100)");
  Serial.println("Presiona Enter para usar valor por defecto");
  
  String brokerip = readSerialLineBlocking("Broker MQTT > ");
  brokerip.toCharArray(mqtt_server, sizeof(mqtt_server));
  
  if (brokerip.length() > 0 && !brokerip.startsWith("mqtt://")) {
    String fullAddr = "mqtt://" + brokerip + ":1883";
    fullAddr.toCharArray(mqtt_server, sizeof(mqtt_server));
  }
  
  Serial.print("Broker configurado: ");
  Serial.println(mqtt_server);
  
  // Configurar pines
  pinMode(TEMP_PIN, INPUT);
  pinMode(PROX_PIN, INPUT_PULLUP); // Pull-up interno para sensor NPN
  
  // Conectar WiFi
  setupWiFi();
  
  // Configurar MQTT
  mqttClient.setURI(mqtt_server);
  mqttClient.setMqttClientName("ESP32_Sensor_Client");
  mqttClient.loopStart();
  
  Serial.println("\nSistema inicializado. Publicando datos...");
  lastTempPublish = millis() - TEMP_PUBLISH_INTERVAL;
}

void loop() {
  unsigned long now = millis();
  
  // Leer sensor de proximidad
  bool proximityDetected = readProximitySensor();
  
  // Publicar cambio de estado de proximidad
  if (proxTriggered) {
    if (mqttClient.isConnected()) {
      publishProximityState(proximityDetected);
    } else {
      Serial.println("MQTT no conectado, no se puede publicar proximidad");
    }
    
    // Mostrar por serial
    Serial.print("Estado proximidad cambiado: ");
    Serial.println(proximityDetected ? "true (OBJETO DETECTADO)" : "false (NO DETECTADO)");
    
    proxTriggered = false;
  }
  
  // Publicación periódica de temperatura
  if (now - lastTempPublish >= TEMP_PUBLISH_INTERVAL) {
    if (mqttClient.isConnected()) {
      publishTemperature();
    } else {
      Serial.println("MQTT no conectado, no se puede publicar temperatura");
    }
    lastTempPublish = now;
  }
  
  // Mantener conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdido, reconectando...");
    setupWiFi();
  }
  
  // Pequeña pausa
  delay(10);
}