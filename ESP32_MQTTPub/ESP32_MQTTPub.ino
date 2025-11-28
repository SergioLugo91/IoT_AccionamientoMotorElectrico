/*
  ESP32 - Publica temperatura por MQTT + publica evento de sensor de proximidad
  (Solicita la IP/dominio del broker por Serial al iniciar)

  - Lee temperatura y publica la temperatura de forma periódica
  - Lee una entrada digital para sensor de proximidad y publica un evento cuando la entrada pasa a HIGH
  - Pide por Serial la dirección (IP o dominio) del broker MQTT al arrancar
  - Usa WiFi + PubSubClient (MQTT)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32MQTTClient.h>

// ========== CONFIG ========
const char* ssid        = "DESKTOP-RTTQ50L 0411";
const char* wifiPass    = "574;Cm23";

char* mqtt_server = "";          // se pedirá por Serial al inicio
const uint16_t mqtt_port = 1883;

const char* tempTopic   = "esp32/temperature";
const char* proxTopic   = "esp32/proximity";

const unsigned long TEMP_PUBLISH_INTERVAL = 1000UL; // publicar temp cada 1s
const unsigned long PROX_DEBOUNCE_MS = 200; // debounce de proximidad en ms
// ==========================

// Pines
const uint8_t TEMP_PIN = 23; // pin entrada analógica del sensor de temperatura
const uint8_t PROX_PIN = 15; // pin entrada digital del sensor de proximidad

WiFiClient wifiClient;  
ESP32MQTTClient mqttClient;

unsigned long lastTempPublish = 0;
unsigned long lastProxPublish = 0;

// ISR-safe flag para indicar trigger de proximidad (se procesará en loop)
volatile bool proxTriggered = false;

void IRAM_ATTR proxISR() {
  proxTriggered = true;
}

// ---------- Helpers ----------
// Required global callback for connection events
void onMqttConnect(esp_mqtt_client_handle_t client) {
  if (mqttClient.isMyTurn(client)) {
    // Subscribe to topics here
    mqttClient.subscribe("test/topic", [](const std::string &payload) {
      Serial.printf("Received: %s\n", payload.c_str());
    });
  }
}

// Required global event handler - ESP-IDF version dependent
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

void setupWiFi() {
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, wifiPass);
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

void publishTemperature() {
  float tempC = AnalogRead(TEMP_PIN);
  Serial.print(tempC);
  char payload[96];
  snprintf(payload, sizeof(payload), "%.2f", tempC);

  bool ok = mqttClient.publish(tempTopic, payload, 0, false);
  Serial.print("Publicado temp: ");
  Serial.print(payload);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FALLA");
}

/*void publishProximityEvent() {
  unsigned long t = millis();
  char payload[96];
  snprintf(payload, sizeof(payload), "{\"proximity\":1,\"ts\":%lu}", t);
  bool ok = client.publish(proxTopic, payload);
  Serial.print("Publicado proximidad: ");
  Serial.print(payload);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FALLA");
}*/

// Lee una línea desde Serial (hasta '\n') y la devuelve recortada.
// Si Serial no está abierta en el monitor, esperar hasta que haya datos.
String readSerialLineBlocking(const char* prompt) {
  Serial.println(prompt);
  String s = "";
  while (s.length() == 0) {
    // esperar hasta que el usuario escriba algo y pulse ENTER
    while (!Serial.available()) {
      delay(10);
    }
    s = Serial.readStringUntil('\n');
    s.trim();
  }
  s = "mqtt://" + s + ":1883";
  return s;
}

// ========== Setup y loop ==========
void setup() {
  Serial.begin(115200);
  delay(100);

  // Pedir al usuario por Serial la dirección del broker MQTT
  Serial.println();
  Serial.println("== Configuración MQTT ==");
  Serial.println("Introduce la IP o dominio del broker MQTT.");

  String brokerip = readSerialLineBlocking("Broker MQTT > ");
  Serial.print("Broker configurado en: ");
  Serial.println(brokerip);

  // Pin sensor proximidad
  pinMode(TEMP_PIN, INPUT);
  pinMode(PROX_PIN, INPUT); // si tu sensor necesita pullup usa INPUT_PULLUP

  // Adjuntar ISR para flanco ascendente (cuando la salida pasa a HIGH)
  attachInterrupt(digitalPinToInterrupt(PROX_PIN), proxISR, RISING);

  // Conectar WiFi
  setupWiFi();

  // Inicializar MQTT y conectar
  mqttClient.setURI(brokerip.c_str());
  mqttClient.setMqttClientName("ESP32_Client");
  mqttClient.loopStart();

  lastTempPublish = millis() - TEMP_PUBLISH_INTERVAL; // publicar inmediatamente al arrancar
}

void loop() {
  // Mantener MQTT y WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdido, reconectando...");
    setupWiFi();
  }
  unsigned long now = millis();

  // Publicación periódica de temperatura
  if (now - lastTempPublish >= TEMP_PUBLISH_INTERVAL) {
    if (mqttClient.isConnected()) publishTemperature();
    lastTempPublish = now;
  }

  // Manejo del trigger de proximidad (evitar publicar desde ISR)
  if (proxTriggered) {
    // Desactivar la bandera lo antes posible
    proxTriggered = false;

    // Leer el pin para asegurarnos que sigue en HIGH (debounce simple)
    if (digitalRead(PROX_PIN) == HIGH) {
      // Evitar rebotes: si publicamos muy recientemente, ignorar
      if (now - lastProxPublish >= PROX_DEBOUNCE_MS) {
        //if (mqttClient.isConnected()) publishProximityEvent();
        lastProxPublish = now;
      } else {
        Serial.println("Proximity trigger ignorado por debounce.");
      }
    } else {
      Serial.println("Proximity trigger detectado pero pin no está HIGH (rebote).");
    }
  }

  // Pequeña pausa
  delay(10);
}