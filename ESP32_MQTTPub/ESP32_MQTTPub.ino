/*
  ESP32 - Publica temperatura por MQTT + publica evento de sensor de proximidad
  (Solicita la IP/dominio del broker por Serial al iniciar)

  - Lee temperatura y publica la temperatura de forma periódica
  - Lee una entrada digital para sensor de proximidad y publica un evento cuando la entrada pasa a HIGH
  - Pide por Serial la dirección (IP o dominio) del broker MQTT al arrancar
  - Usa WiFi + PubSubClient (MQTT)
*/

#include <WiFi.h>
#include <PubSubClient.h>

// ========== CONFIG ========
const char* ssid        = "DESKTOP-RTTQ50L 0411";
const char* wifiPass    = "574;Cm23";

String mqtt_server = "";          // se pedirá por Serial al inicio
const uint16_t mqtt_port = 1883;

const char* tempTopic   = "home/esp32/temperature";
const char* proxTopic   = "home/esp32/proximity";

const unsigned long TEMP_PUBLISH_INTERVAL = 1000UL; // publicar temp cada 1s
const unsigned long PROX_DEBOUNCE_MS = 200; // debounce de proximidad en ms
// ==========================

// Pines
const uint8_t TEMP_PIN = A0; // pin entrada analógica del sensor de temperatura
const uint8_t PROX_PIN = 15; // pin entrada digital del sensor de proximidad

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastTempPublish = 0;
unsigned long lastProxPublish = 0;

// ISR-safe flag para indicar trigger de proximidad (se procesará en loop)
volatile bool proxTriggered = false;

void IRAM_ATTR proxISR() {
  proxTriggered = true;
}

// ---------- Helpers ----------
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

// Reconexión MQTT usando mqtt_server (String)
void reconnectMQTT() {
  if (client.connected()) return;
  if (mqtt_server.length() == 0) {
    Serial.println("Broker MQTT no configurado.");
    return;
  }

  Serial.print("Conectando al broker MQTT: ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.println(mqtt_port);

  client.setServer(mqtt_server.c_str(), mqtt_port);
  String clientId = "ESP32-";
  clientId += String((uint32_t)esp_random(), HEX);

  bool ok;
  if (strlen(mqtt_user) > 0) ok = client.connect(clientId.c_str(), mqtt_user, mqtt_pass);
  else ok = client.connect(clientId.c_str());

  if (ok) {
    Serial.println("Conectado al broker MQTT");
  } else {
    Serial.print("Fallo al conectar MQTT, rc=");
    Serial.println(client.state());
    delay(3000);
  }
}

void publishTemperature() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: DS18B20 desconectado");
    return;
  }

  char payload[96];
  unsigned long t = millis();
  snprintf(payload, sizeof(payload), "{\"temperature\":%.2f,\"ts\":%lu}", tempC, t);

  bool ok = client.publish(tempTopic, payload, true);
  Serial.print("Publicado temp: ");
  Serial.print(payload);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FALLA");
}

void publishProximityEvent() {
  unsigned long t = millis();
  char payload[96];
  snprintf(payload, sizeof(payload), "{\"proximity\":1,\"ts\":%lu}", t);
  bool ok = client.publish(proxTopic, payload);
  Serial.print("Publicado proximidad: ");
  Serial.print(payload);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FALLA");
}

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
  return s;
}

// ========== Setup y loop ==========
void setup() {
  Serial.begin(115200);
  delay(100);

  // Pedir al usuario por Serial la dirección del broker MQTT
  Serial.println();
  Serial.println("== Configuración MQTT ==");
  Serial.println("Introduce la IP o dominio del broker MQTT y pulsa ENTER.");
  Serial.println("Ejemplo: 192.168.1.20   o   broker.example.com");
  Serial.println("(No continuar hasta que escribas la dirección)");

  mqtt_server = readSerialLineBlocking("Broker MQTT > ");
  Serial.print("Broker configurado en: ");
  Serial.println(mqtt_server);

  // Inicializar sensor de temperatura
  sensors.begin();

  // Pin sensor proximidad
  pinMode(PROX_PIN, INPUT); // si tu sensor necesita pullup usa INPUT_PULLUP

  // Adjuntar ISR para flanco ascendente (cuando la salida pasa a HIGH)
  attachInterrupt(digitalPinToInterrupt(PROX_PIN), proxISR, RISING);

  // Conectar WiFi
  setupWiFi();

  // Inicializar MQTT y conectar
  client.setServer(mqtt_server.c_str(), mqtt_port);
  reconnectMQTT();

  lastTempPublish = millis() - TEMP_PUBLISH_INTERVAL; // publicar inmediatamente al arrancar
}

void loop() {
  // Mantener MQTT y WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdido, reconectando...");
    setupWiFi();
  }
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  unsigned long now = millis();

  // Publicación periódica de temperatura
  if (now - lastTempPublish >= TEMP_PUBLISH_INTERVAL) {
    if (client.connected()) publishTemperature();
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
        if (client.connected()) publishProximityEvent();
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