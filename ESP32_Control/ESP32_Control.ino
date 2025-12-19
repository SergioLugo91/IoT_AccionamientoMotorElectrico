/*
  ESP32 - Control web de motor Dahlander de dos velocidades
  Implementación usando solo la librería WiFi (WiFiServer) y parsing manual de HTTP
  - Página web con botones:
      * Encender velocidad lenta
      * Cambiar a velocidad rápida
      * Apagar motor
      * Control automático (MQTT)
  - Comportamiento:
      Velocidad lenta: activa relés L1 y L2 (conexión delta)
      Velocidad rápida: activa relés L1, L3 y L4 (conexión doble estrella)
  - Control automático:
      * Si temperatura > umbral -> parar motor
      * Si sensor proximidad detecta objeto -> cambiar a velocidad lenta
*/

#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32MQTTClient.h>

// --- CONFIGURACIÓN ---
const char* ssid = "DESKTOP-RTTQ50L 0411";
const char* password = "574;Cm23";
const uint16_t HTTP_PORT = 80;
char* mqtt_server = "";          // se pedirá por Serial al inicio
const uint16_t mqtt_port = 1883;

// Tópicos mqtt:
const char* tempTopic = "esp32/temperature";
float lastTemp = 0.0;
const char* proxTopic = "esp32/proximity";
bool lastProx = false;

// Umbrales para control automático
const float TEMP_UMBRAL = 60.0;  // °C - temperatura máxima permitida
const bool PROX_ACTIVA_BAJA = true; // true = cuando detecta objeto, cambiar a baja velocidad

// Pines I2C:
const uint8_t I2C_SDA_PIN = 17;
const uint8_t I2C_SCL_PIN = 16;

// Dirección I2C del módulo LCD
const uint8_t LCD_I2C_ADDR = 0x27;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);

// Pines de relés para motor Dahlander (ajusta según tu conexión)
const uint8_t REL_L1_PIN = 18;  // Relé principal
const uint8_t REL_L2_PIN = 19;  // Para velocidad lenta (delta)
const uint8_t REL_L3_PIN = 23;  // Para velocidad rápida (doble estrella)
const uint8_t REL_L4_PIN = 5;   // Para velocidad rápida (doble estrella)

// Si tu módulo de relés es activo ALTO pon true, si es activo BAJO pon false
const bool RELAY_ACTIVE_HIGH = false;

WiFiServer server(HTTP_PORT);

// Estados del sistema
enum SystemMode {
  MODE_MANUAL = 0,
  MODE_AUTO = 1
};

enum MotorState {
  STATE_OFF = 0,
  STATE_SLOW = 1,    // Velocidad lenta (delta)
  STATE_FAST = 2     // Velocidad rápida (doble estrella)
};

SystemMode systemMode = MODE_MANUAL;
MotorState motorState = STATE_OFF;

// Estado lógico de cada relé
bool stateRelL1 = false;
bool stateRelL2 = false;
bool stateRelL3 = false;
bool stateRelL4 = false;

// Variables para control automático
bool autoControlActive = false;
unsigned long lastAutoCheck = 0;
const unsigned long AUTO_CHECK_INTERVAL = 1000; // ms

void relayWrite(uint8_t pin, bool on) {
  if (RELAY_ACTIVE_HIGH) digitalWrite(pin, on ? HIGH : LOW);
  else digitalWrite(pin, on ? LOW : HIGH);
}

void applyRelayStates() {
  relayWrite(REL_L1_PIN, stateRelL1);
  relayWrite(REL_L2_PIN, stateRelL2);
  relayWrite(REL_L3_PIN, stateRelL3);
  relayWrite(REL_L4_PIN, stateRelL4);
  updateLCD();
}

void stopMotor() {
  motorState = STATE_OFF;
  stateRelL1 = false;
  stateRelL2 = false;
  stateRelL3 = false;
  stateRelL4 = false;
  applyRelayStates();
  Serial.println("Motor detenido.");
}

void setSlowSpeed() {
  if (motorState == STATE_SLOW) return;
  
  // Configuración para velocidad lenta (delta): L1 + L2
  motorState = STATE_SLOW;
  stateRelL1 = true;
  stateRelL2 = true;
  stateRelL3 = false;
  stateRelL4 = false;
  
  applyRelayStates();
  Serial.println("Motor en velocidad lenta.");
}

void setFastSpeed() {
  if (motorState == STATE_FAST) return;
  
  // Configuración para velocidad rápida (doble estrella): L1 + L3 + L4
  motorState = STATE_FAST;
  stateRelL1 = true;
  stateRelL2 = false;
  stateRelL3 = true;
  stateRelL4 = true;
  
  applyRelayStates();
  Serial.println("Motor en velocidad rápida.");
}

// Construye JSON de estado
String buildStatusJson() {
  String modeName = (systemMode == MODE_MANUAL) ? "MANUAL" : "AUTO";
  String stateName;
  switch(motorState) {
    case STATE_OFF: stateName = "OFF"; break;
    case STATE_SLOW: stateName = "SLOW"; break;
    case STATE_FAST: stateName = "FAST"; break;
    default: stateName = "UNKNOWN"; break;
  }
  
  String tempStr = String(lastTemp, 1);
  String proxStr = lastProx ? "DETECTADO" : "NO_DETECTADO";
  
  String resp = "{";
  resp += "\"mode\":\"" + modeName + "\",";
  resp += "\"state\":\"" + stateName + "\",";
  resp += "\"temperature\":" + tempStr + ",";
  resp += "\"proximity\":\"" + proxStr + "\",";
  resp += "\"rel_l1\":" + String(stateRelL1 ? "true" : "false") + ",";
  resp += "\"rel_l2\":" + String(stateRelL2 ? "true" : "false") + ",";
  resp += "\"rel_l3\":" + String(stateRelL3 ? "true" : "false") + ",";
  resp += "\"rel_l4\":" + String(stateRelL4 ? "true" : "false");
  resp += "}";
  return resp;
}

// Decodificador URL simple
String urlDecode(const String &src) {
  String ret = "";
  char c;
  for (int i = 0; i < src.length(); i++) {
    c = src[i];
    if (c == '+') ret += ' ';
    else if (c == '%' && i + 2 < src.length()) {
      String hex = src.substring(i + 1, i + 3);
      char decoded = (char) strtol(hex.c_str(), NULL, 16);
      ret += decoded;
      i += 2;
    } else ret += c;
  }
  return ret;
}

void parseQueryParams(const String &query, String &cmd) {
  cmd = "";
  int idx = 0;
  while (idx < query.length()) {
    int amp = query.indexOf('&', idx);
    String part;
    if (amp >= 0) {
      part = query.substring(idx, amp);
      idx = amp + 1;
    } else {
      part = query.substring(idx);
      idx = query.length();
    }
    int eq = part.indexOf('=');
    if (eq >= 0) {
      String k = urlDecode(part.substring(0, eq));
      String v = urlDecode(part.substring(eq + 1));
      if (k == "cmd") cmd = v;
    }
  }
}

// Actualizar LCD
void updateLCD() {
  String line1, line2;
  
  // Línea 1: Modo y estado
  if (systemMode == MODE_AUTO) {
    line1 = "AUTO ";
  } else {
    line1 = "MANUAL ";
  }
  
  switch(motorState) {
    case STATE_OFF: line1 += "OFF"; break;
    case STATE_SLOW: line1 += "SLOW"; break;
    case STATE_FAST: line1 += "FAST"; break;
  }
  
  // Línea 2: Datos MQTT
  line2 = "T:" + String(lastTemp, 1) + "C ";
  line2 = line2 + "P:" + (lastProx ? "SI" : "NO");
  
  // Asegurar longitud 16
  while (line1.length() < 16) line1 += " ";
  while (line2.length() < 16) line2 += " ";
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// Manejo de petición HTTP
void handleClient(WiFiClient &client) {
  String requestLine = "";
  unsigned long startMillis = millis();
  const unsigned long clientTimeout = 5000;
  
  while (client.connected() && (millis() - startMillis) <= clientTimeout) {
    if (client.available()) {
      char c = client.read();
      requestLine += c;
      int len = requestLine.length();
      if (len >= 2 && requestLine.substring(len - 2) == "\r\n") {
        requestLine = requestLine.substring(0, len - 2);
        break;
      }
    }
  }
  
  if (requestLine.length() == 0) {
    client.stop();
    return;
  }
  
  int firstSpace = requestLine.indexOf(' ');
  int secondSpace = -1;
  if (firstSpace >= 0) secondSpace = requestLine.indexOf(' ', firstSpace + 1);
  if (firstSpace < 0 || secondSpace < 0) {
    client.stop();
    return;
  }
  
  String pathAndQuery = requestLine.substring(firstSpace + 1, secondSpace);
  String path = pathAndQuery;
  String query = "";
  int qpos = pathAndQuery.indexOf('?');
  if (qpos >= 0) {
    path = pathAndQuery.substring(0, qpos);
    query = pathAndQuery.substring(qpos + 1);
  }
  
  if (path == "/control") {
    String cmd;
    if (query.length() > 0) parseQueryParams(query, cmd);
    
    Serial.printf("Control request cmd='%s'\n", cmd.c_str());
    
    if (cmd == "stop") {
      systemMode = MODE_MANUAL;
      stopMotor();
    } else if (cmd == "slow") {
      systemMode = MODE_MANUAL;
      setSlowSpeed();
    } else if (cmd == "fast") {
      systemMode = MODE_MANUAL;
      setFastSpeed();
    } else if (cmd == "auto") {
      systemMode = MODE_AUTO;
      autoControlActive = true;
      Serial.println("Modo automático activado.");
    }
    
    String resp = buildStatusJson();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println();
    client.println(resp);
  }
  else if (path == "/status") {
    String resp = buildStatusJson();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println("Access-Control-Allow-Origin: *");
    client.println();
    client.println(resp);
  }
  else if (path == "/" || path == "/index.html") {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    client.println("<!doctype html><html><head><meta charset='utf-8'>");
    client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    client.println("<title>Control Motor Dahlander - ESP32</title>");
    client.println("<style>");
    client.println("body{font-family:Arial;max-width:500px;margin:20px auto;padding:20px;text-align:center}");
    client.println(".controls{margin:20px 0}");
    client.println(".btn{display:inline-block;padding:15px 25px;margin:10px;font-size:18px;");
    client.println("border:none;border-radius:5px;cursor:pointer;color:white;font-weight:bold}");
    client.println(".slow-btn{background-color:#4CAF50}");
    client.println(".fast-btn{background-color:#2196F3}");
    client.println(".stop-btn{background-color:#f44336}");
    client.println(".auto-btn{background-color:#FF9800}");
    client.println(".status-container{margin:30px 0;padding:20px;border:2px solid #ddd;border-radius:10px}");
    client.println(".status-row{margin:10px 0;display:flex;justify-content:space-between}");
    client.println(".status-label{font-weight:bold}");
    client.println(".status-value{color:#333}");
    client.println(".on{color:green}.off{color:red}");
    client.println("</style></head><body>");
    client.println("<h1>Control Motor Dahlander</h1>");
    client.println("<h3>Dos Velocidades</h3>");
    
    client.println("<div class='controls'>");
    client.println("<button class='btn slow-btn' onclick=\"control('slow')\">Velocidad Lenta</button>");
    client.println("<button class='btn fast-btn' onclick=\"control('fast')\">Velocidad Rápida</button><br>");
    client.println("<button class='btn stop-btn' onclick=\"control('stop')\">Apagar Motor</button>");
    client.println("<button class='btn auto-btn' onclick=\"control('auto')\">Control Automático</button>");
    client.println("</div>");
    
    client.println("<div class='status-container'>");
    client.println("<h3>Estado del Sistema</h3>");
    client.println("<div class='status-row'><span class='status-label'>Modo:</span><span class='status-value' id='mode'>-</span></div>");
    client.println("<div class='status-row'><span class='status-label'>Estado Motor:</span><span class='status-value' id='state'>-</span></div>");
    client.println("<div class='status-row'><span class='status-label'>Temperatura:</span><span class='status-value' id='temp'>-</span></div>");
    client.println("<div class='status-row'><span class='status-label'>Sensor Proximidad:</span><span class='status-value' id='prox'>-</span></div>");
    client.println("<div class='status-row'><span class='status-label'>Relé L1:</span><span class='status-value' id='rel1'>-</span></div>");
    client.println("<div class='status-row'><span class='status-label'>Relé L2:</span><span class='status-value' id='rel2'>-</span></div>");
    client.println("<div class='status-row'><span class='status-label'>Relé L3:</span><span class='status-value' id='rel3'>-</span></div>");
    client.println("<div class='status-row'><span class='status-label'>Relé L4:</span><span class='status-value' id='rel4'>-</span></div>");
    client.println("</div>");
    
    client.println("<script>");
    client.println("async function control(cmd){");
    client.println("  const res = await fetch('/control?cmd=' + encodeURIComponent(cmd));");
    client.println("  const data = await res.json();");
    client.println("  updateDisplay(data);");
    client.println("}");
    
    client.println("async function updateStatus(){");
    client.println("  try{");
    client.println("    const res = await fetch('/status');");
    client.println("    const data = await res.json();");
    client.println("    updateDisplay(data);");
    client.println("  } catch(e){ console.error('Error:', e); }");
    client.println("}");
    
    client.println("function updateDisplay(data){");
    client.println("  document.getElementById('mode').textContent = data.mode;");
    client.println("  document.getElementById('state').textContent = data.state;");
    client.println("  document.getElementById('temp').textContent = data.temperature + ' °C';");
    client.println("  document.getElementById('prox').textContent = data.proximity;");
    client.println("  document.getElementById('rel1').innerHTML = data.rel_l1 ? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>';");
    client.println("  document.getElementById('rel2').innerHTML = data.rel_l2 ? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>';");
    client.println("  document.getElementById('rel3').innerHTML = data.rel_l3 ? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>';");
    client.println("  document.getElementById('rel4').innerHTML = data.rel_l4 ? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>';");
    client.println("}");
    
    client.println("setInterval(updateStatus, 1000);");
    client.println("updateStatus();");
    client.println("</script>");
    client.println("</body></html>");
  }
  else {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Not found");
  }
  
  delay(1);
  client.stop();
}

// Funciones MQTT
ESP32MQTTClient mqttClient;

void onMqttConnect(esp_mqtt_client_handle_t client) {
  if (mqttClient.isMyTurn(client)) {
    mqttClient.subscribe(tempTopic, [](const std::string &payload) {
      lastTemp = atof(payload.c_str());
      Serial.printf("Temperatura recibida: %.1f °C\n", lastTemp);
    });
    
    mqttClient.subscribe(proxTopic, [](const std::string &payload) {
      lastProx = (payload == "1" || payload == "true" || payload == "DETECTADO");
      Serial.printf("Proximidad: %s\n", lastProx ? "DETECTADO" : "NO DETECTADO");
    });
    
    Serial.println("Suscrito a tópicos MQTT");
  }
}

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

String readSerialLineBlocking(const char* prompt) {
  Serial.println(prompt);
  String s = "";
  while (s.length() == 0) {
    while (!Serial.available()) {
      delay(10);
    }
    s = Serial.readStringUntil('\n');
    s.trim();
  }
  s = "mqtt://" + s + ":1883";
  return s;
}

// Control automático basado en sensores MQTT
void autoControl() {
  if (systemMode != MODE_AUTO) return;
  
  // Control por temperatura: si supera umbral, parar motor
  if (lastTemp > TEMP_UMBRAL) {
    Serial.printf("Temperatura crítica (%.1f °C > %.1f °C). Parando motor.\n", lastTemp, TEMP_UMBRAL);
    stopMotor();
    systemMode = MODE_MANUAL; // Cambiar a manual después de parada de emergencia
    return;
  }
  
  // Control por proximidad
  if (PROX_ACTIVA_BAJA) {
    if (lastProx && motorState == STATE_FAST) {
      // Si detecta objeto y está en velocidad rápida, cambiar a lenta
      Serial.println("Objeto detectado. Cambiando a velocidad lenta.");
      setSlowSpeed();
      return;
    } else if (!lastProx && motorState == STATE_SLOW) {
      // Si no hay objeto y está en lenta, cambiar a rápida
      Serial.println("Sin objetos. Cambiando a velocidad rápida.");
      setFastSpeed();
    }
  }
  
  // Si está apagado y no hay condiciones críticas, encender en velocidad lenta
  if (motorState == STATE_OFF && lastTemp <= TEMP_UMBRAL) {
    setSlowSpeed();
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // Inicializar I2C y LCD
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Iniciando...");
  delay(500);
  
  // Configurar pines de relés
  pinMode(REL_L1_PIN, OUTPUT);
  pinMode(REL_L2_PIN, OUTPUT);
  pinMode(REL_L3_PIN, OUTPUT);
  pinMode(REL_L4_PIN, OUTPUT);
  
  // Asegurar que estén apagados
  stopMotor();
  
  // Conectar WiFi
  Serial.printf("Conectando a SSID: %s\n", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nTiempo de conexión agotado. Reintentando...");
      start = millis();
      WiFi.begin(ssid, password);
    }
  }
  Serial.println();
  Serial.print("Conectado! IP: ");
  Serial.println(WiFi.localIP());
  
  updateLCD();
  
  server.begin();
  Serial.printf("Servidor HTTP iniciado en puerto %d\n", HTTP_PORT);
  
  // Configurar MQTT
  Serial.println("== Configuración MQTT ==");
  Serial.println("Introduce la IP o dominio del broker MQTT:");
  
  String brokerip = readSerialLineBlocking("Broker MQTT > ");
  Serial.print("Broker configurado en: ");
  Serial.println(brokerip);
  
  mqttClient.setURI(brokerip.c_str());
  mqttClient.setMqttClientName("ESP32_Dahlander");
  mqttClient.loopStart();
  
  Serial.println("Sistema listo. Accede a la IP en tu navegador.");
}

void loop() {
  // Control automático periódico
  if (millis() - lastAutoCheck >= AUTO_CHECK_INTERVAL) {
    autoControl();
    lastAutoCheck = millis();
  }
  
  // Manejar clientes HTTP
  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
  }
  
  // Actualizar LCD periódicamente
  static unsigned long lastLCDUpdate = 0;
  if (millis() - lastLCDUpdate >= 1000) {
    updateLCD();
    lastLCDUpdate = millis();
  }

  // Asegurar que si el motor está apagado el resto de relés estén apagados
  if (!stateRelL1) {
    stateRelL2 = false;
    stateRelL3 = false;
    stateRelL4 = false;
    applyRelayStates();
  }
  
  delay(10);
}