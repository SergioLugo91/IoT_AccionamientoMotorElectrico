
/*
  ESP32 - Control web de 4 relés (Estrella/Triángulo + Sentido CW/CCW + Apagar)
  Implementación usando solo la librería WiFi (WiFiServer) y parsing manual de HTTP
  - Página web con botones:
      * Encender (Horario)
      * Encender (Antihorario)
      * Apagar
  - Comportamiento:
      Al "encender" (CW o CCW) primero se activa:
        1) el relé del sentido (CW o CCW)
        2) el relé ESTRELLA
      tras 2 segundos:
        - se apaga ESTRELLA y se enciende TRIÁNGULO
  - Evita activar CW y CCW simultáneamente.
  - No usa WebServer ni otras librerías HTTP, solo WiFi.h
*/

#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- CONFIGURACIÓN (ajusta) ---
const char* ssid = "DESKTOP-RTTQ50L 0411";
const char* password = "574;Cm23";
const uint16_t HTTP_PORT = 80;

// Pines I2C:
const uint8_t I2C_SDA_PIN = 17; // según tu petición
const uint8_t I2C_SCL_PIN = 16; // según tu petición

// Dirección I2C del módulo LCD (0x27 o 0x3F son comunes). Cámbiala si no se muestra.
const uint8_t LCD_I2C_ADDR = 0x27;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2); // 16x2

// Pines de relés (ajusta según tu conexión física)
const uint8_t REL_CW_PIN    = 18; // relé para sentido horario (CW)
const uint8_t REL_CCW_PIN   = 19; // relé para sentido antihorario (CCW)
const uint8_t REL_STAR_PIN  = 23; // relé Estrella (Y)
const uint8_t REL_DELTA_PIN = 5; // relé Triángulo (Δ)

// Si tu módulo de relés es activo ALTO pon true, si es activo BAJO pon false
const bool RELAY_ACTIVE_HIGH = false;

// Tiempo entre activar ESTRELLA y cambiar a TRIÁNGULO (ms)
const unsigned long STAR_TO_DELTA_MS = 2000UL;
// -------------------------------

WiFiServer server(HTTP_PORT);

// Estados del motor
enum MotorPhase { PHASE_IDLE = 0, PHASE_STARTING = 1, PHASE_RUNNING = 2 };
MotorPhase motorPhase = PHASE_IDLE;
int currentDir = 0; // 0 = none, 1 = CW, 2 = CCW
unsigned long phaseStartMillis = 0;

// Estado lógico de cada relé (true = activado lógico)
bool stateRelCW = false;
bool stateRelCCW = false;
bool stateRelStar = false;
bool stateRelDelta = false;

void relayWrite(uint8_t pin, bool on) {
  if (RELAY_ACTIVE_HIGH) digitalWrite(pin, on ? HIGH : LOW);
  else digitalWrite(pin, on ? LOW : HIGH);
}

void applyRelayStates() {
  relayWrite(REL_CW_PIN, stateRelCW);
  relayWrite(REL_CCW_PIN, stateRelCCW);
  relayWrite(REL_STAR_PIN, stateRelStar);
  relayWrite(REL_DELTA_PIN, stateRelDelta);
  updateLCD(); // refrescar pantalla cada vez que cambian relés
}

void stopMotor() {
  currentDir = 0;
  motorPhase = PHASE_IDLE;
  stateRelCW = false;
  stateRelCCW = false;
  stateRelStar = false;
  stateRelDelta = false;
  applyRelayStates();
  Serial.println("Motor detenido.");
}

void startMotor(int dir) {
  if (motorPhase != PHASE_IDLE && currentDir == dir) return;
  if (currentDir != 0 && currentDir != dir) {
    stopMotor();
    delay(100);
  }
  currentDir = dir;
  motorPhase = PHASE_STARTING;
  phaseStartMillis = millis();

  stateRelDelta = false;
  stateRelStar = true;
  stateRelCW = (dir == 1);
  stateRelCCW = (dir == 2);

  applyRelayStates();
  Serial.printf("startMotor dir=%d (CW=%d CCW=%d STAR=1)\n", dir, stateRelCW, stateRelCCW);
}

// Construye JSON de estado
String buildStatusJson() {
  String phaseName = (motorPhase == PHASE_IDLE) ? "IDLE" : (motorPhase == PHASE_STARTING ? "STARTING" : "RUNNING");
  String dirName = (currentDir == 0) ? "NONE" : (currentDir == 1 ? "CW" : "CCW");

  String resp = "{";
  resp += "\"phase\":\"" + phaseName + "\",";
  resp += "\"direction\":\"" + dirName + "\",";
  resp += "\"rel_cw\":" + String(stateRelCW ? "true" : "false") + ",";
  resp += "\"rel_ccw\":" + String(stateRelCCW ? "true" : "false") + ",";
  resp += "\"rel_star\":" + String(stateRelStar ? "true" : "false") + ",";
  resp += "\"rel_delta\":" + String(stateRelDelta ? "true" : "false");
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

void parseQueryParams(const String &query, String &cmd, String &dir) {
  cmd = "";
  dir = "";
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
      else if (k == "dir") dir = v;
    }
  }
}

// --- LCD: mostrar IP y estado ---
void updateLCD() {
  // Protege por si el LCD no está inicializado
  // Línea 0: IP, Línea 1: estado (OFF / ON CW Y / ON CCW Δ)
  String ip = WiFi.localIP().toString();
  String line1 = ip;
  // limitar a 16 caracteres
  if (line1.length() > 16) line1 = line1.substring(0, 16);

  String line2;
  if (motorPhase == PHASE_IDLE) {
    line2 = "OFF";
  } else {
    String dir = (currentDir == 1) ? "CW" : (currentDir == 2 ? "CCW" : "UNK");
    String conn = stateRelStar ? "Y" : (stateRelDelta ? "D" : "??"); // Y=Estrella, D=Delta (triángulo)
    // mostrar Y o Δ: usamos "Y" para estrella y "D" para triángulo por limitación de caracteres
    // Construir: "ON CW Y" por ejemplo
    line2 = "ON " + dir + " ";
    if (stateRelStar) line2 += "Y";
    else if (stateRelDelta) line2 += "D";
    else line2 += "-";
  }
  // Asegurar longitud 16 llenando espacios
  while (line1.length() < 16) line1 += " ";
  while (line2.length() < 16) line2 += " ";

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// Manejo de petición HTTP simple (lee primera línea y parsea)
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

  Serial.println("RequestLine: " + requestLine);

  int firstSpace = requestLine.indexOf(' ');
  int secondSpace = -1;
  if (firstSpace >= 0) secondSpace = requestLine.indexOf(' ', firstSpace + 1);
  if (firstSpace < 0 || secondSpace < 0) {
    client.stop();
    return;
  }

  String pathAndQuery = requestLine.substring(firstSpace + 1, secondSpace);
  Serial.println("PathAndQuery: " + pathAndQuery);

  String path = pathAndQuery;
  String query = "";
  int qpos = pathAndQuery.indexOf('?');
  if (qpos >= 0) {
    path = pathAndQuery.substring(0, qpos);
    query = pathAndQuery.substring(qpos + 1);
  }

  if (path == "/control") {
    String cmd, dir;
    if (query.length() > 0) parseQueryParams(query, cmd, dir);
    Serial.printf("Control request cmd='%s' dir='%s'\n", cmd.c_str(), dir.c_str());
    if (cmd == "stop") {
      stopMotor();
    } else if (cmd == "start") {
      if (dir == "cw") startMotor(1);
      else if (dir == "ccw") startMotor(2);
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
    client.println("<title>Control Motor - ESP32</title>");
    client.println("<style>body{font-family:Arial;max-width:480px;margin:20px auto;padding:10px;text-align:center}button{padding:12px 18px;margin:6px;font-size:16px}.status{margin-top:12px;padding:8px;border:1px solid #ccc}.on{color:green;font-weight:bold}.off{color:darkred;font-weight:bold}</style></head><body>");
    client.println("<h2>Control Motor (Estrella/Triángulo)</h2>");
    client.println("<div>");
    client.println("<button onclick=\"start('cw')\">Encender (Horario)</button>");
    client.println("<button onclick=\"start('ccw')\">Encender (Antihorario)</button>");
    client.println("<button onclick=\"stopMotor()\">Apagar</button>");
    client.println("</div>");
    client.println("<div class='status' id='status'>Cargando estado...</div>");
    client.println("<script>");
    client.println("async function controlQuery(cmd, dir){");
    client.println("  let url = '/control?cmd=' + encodeURIComponent(cmd);");
    client.println("  if(dir) url += '&dir=' + encodeURIComponent(dir);");
    client.println("  const res = await fetch(url); return await res.json();");
    client.println("}");
    client.println("function start(d){ controlQuery('start', d).then(updateStatus).catch(()=>alert('Error')); }");
    client.println("function stopMotor(){ controlQuery('stop').then(updateStatus).catch(()=>alert('Error')); }");
    client.println("async function updateStatus(){");
    client.println("  try{ const res = await fetch('/status'); const s = await res.json();");
    client.println("    let html = '<b>Fase:</b> ' + s.phase + '<br>'; html += '<b>Dirección:</b> ' + s.direction + '<br>'; html += '<b>Relés:</b><br>'; html += ' CW: ' + (s.rel_cw? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>') + '<br>'; html += ' CCW: ' + (s.rel_ccw? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>') + '<br>'; html += ' ESTRELLA: ' + (s.rel_star? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>') + '<br>'; html += ' TRIÁNGULO: ' + (s.rel_delta? '<span class=\"on\">ON</span>' : '<span class=\"off\">OFF</span>') + '<br>'; document.getElementById('status').innerHTML = html;");
    client.println("  } catch(e){ document.getElementById('status').innerText = 'Error obteniendo estado'; }");
    client.println("}");
    client.println("setInterval(updateStatus, 1000); updateStatus();");
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

void setup() {
  Serial.begin(115200);
  delay(100);

  // Inicializar I2C en los pines solicitados
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // Inicializar LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Iniciando...");
  delay(500);

  // Pines relés
  pinMode(REL_CW_PIN, OUTPUT);
  pinMode(REL_CCW_PIN, OUTPUT);
  pinMode(REL_STAR_PIN, OUTPUT);
  pinMode(REL_DELTA_PIN, OUTPUT);

  // Asegurar que estén apagados al inicio
  stateRelCW = stateRelCCW = stateRelStar = stateRelDelta = false;
  applyRelayStates();

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

  // Mostrar IP en LCD
  updateLCD();

  server.begin();
  Serial.printf("Servidor HTTP iniciado en puerto %d\n", HTTP_PORT);
}

void loop() {
  // Manejo no bloqueante de la secuencia ESTRELLA->TRIÁNGULO
  if (motorPhase == PHASE_STARTING) {
    unsigned long now = millis();
    if (now - phaseStartMillis >= STAR_TO_DELTA_MS) {
      stateRelStar = false;
      stateRelDelta = true;
      motorPhase = PHASE_RUNNING;
      applyRelayStates();
      Serial.println("Cambio ESTRELLA -> TRIÁNGULO completado.");
    }
  }

  // Evitar ambos sentidos activos
  if (stateRelCW && stateRelCCW) {
    Serial.println("Conflicto de direcciones detectado: apagando ambos por seguridad.");
    stopMotor();
  }

  WiFiClient client = server.available();
  if (client) {
    Serial.println("Nuevo cliente conectado.");
    handleClient(client);
    Serial.println("Cliente desconectado.");
  }

  // Refrescar LCD periódicamente (por si cambia IP o estado externo)
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 1000) {
    updateLCD();
    lastLCD = millis();
  }

  delay(10);
}