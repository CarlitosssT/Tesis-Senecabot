/*
 * ESP32: 16 servos por PCA9685, controlados por WiFi (TCP) y tambien por Serial.
 *
 * Es eris_servo_ina sin el INA219, con el mismo protocolo de texto de
 * ErisServoDriver. Los comandos llegan por un socket TCP (puerto TCP_PORT) o por
 * USB-Serial; cada respuesta se envia por el mismo canal por el que llego el comando.
 *
 * Librerias (Library Manager): "Adafruit PWM Servo Driver Library".
 * Board: ESP32 Dev Module. Baud: 921600.
 *
 * El ESP32 crea su propia red WiFi (SoftAP): conecta el PC a la red AP_SSID con
 * la clave AP_PASS y usa la IP 192.168.4.1. No necesita router ni internet
 * (Windows avisara "sin internet": es normal, mantener la conexion).
 *
 * Cableado:
 *   PCA9685 -> I2C: SDA=GPIO21, SCL=GPIO22, direccion 0x40
 *
 * Protocolo: lineas de texto terminadas en '\n', tokens separados por espacio.
 *   X <ch> <ang>            mueve un canal (0-15) al angulo (0-180), inmediato
 *   X <a0> ... <a15>        mueve los 16 canales, inmediato
 *   Y <ch> <ang> | Y <a0>..<a15>   igual pero con rampa (SMOOTH_SPEED_DPS)
 *   INFO                    info del firmware (incluye IP y RSSI)
 *   ON / OFF                LED de la placa
 * Cada comando responde exactamente una linea ("OK ..." o "Error: ...").
 *
 * Prueba rapida desde Python:
 *   import socket
 *   s = socket.create_connection(("192.168.4.1", 8888))
 *   s.sendall(b"X 0 90\n"); print(s.recv(100))
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---- WiFi: el ESP32 crea su propia red (SoftAP) ----
const char* AP_SSID = "ERIS_SERVO";
const char* AP_PASS = "eris12345";  // minimo 8 caracteres (o "" para red abierta)
const int AP_CHANNEL = 6;
const uint16_t TCP_PORT = 8888;
// IP del ESP32 en su red: 192.168.4.1 (por defecto de SoftAP)

const unsigned long SERIAL_BAUD = 921600;

const int PCA_SDA = 21, PCA_SCL = 22;
const uint8_t PCA9685_ADDR = 0x40;

const int LED_PIN = 2;

const int NUM_SERVOS = 16;
const float ANGLE_MIN = 0, ANGLE_MAX = 180;
const int SERVO_MIN_US = 500, SERVO_MAX_US = 2400;
const float SMOOTH_SPEED_DPS = 90.0;  // velocidad de la rampa de Y (grados/s)
const unsigned long SERVO_UPDATE_MS = 20;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR, Wire);
WiFiServer server(TCP_PORT);
WiFiClient client;

// Canal de salida del comando en curso (Serial o el cliente TCP).
Print* out = &Serial;

float currentAngle[NUM_SERVOS];
float targetAngle[NUM_SERVOS];
bool known[NUM_SERVOS];  // false hasta el primer comando: evita saltos al arrancar

unsigned long lastServoMs = 0;

String serialLine, tcpLine;
const size_t MAX_LINE = 512;

float clampAngle(float a) {
  return a < ANGLE_MIN ? ANGLE_MIN : (a > ANGLE_MAX ? ANGLE_MAX : a);
}

void writeServo(int ch, float angle) {
  float us = SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  pwm.writeMicroseconds(ch, (uint16_t)(us + 0.5f));
}

void setTarget(int ch, float angle, bool smooth) {
  angle = clampAngle(angle);
  targetAngle[ch] = angle;
  if (!smooth || !known[ch]) {
    currentAngle[ch] = angle;
    known[ch] = true;
    writeServo(ch, angle);
  }
}

void updateSmooth() {
  unsigned long now = millis();
  if (now - lastServoMs < SERVO_UPDATE_MS) return;
  float step = SMOOTH_SPEED_DPS * (now - lastServoMs) / 1000.0f;
  lastServoMs = now;
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    if (!known[ch] || currentAngle[ch] == targetAngle[ch]) continue;
    float diff = targetAngle[ch] - currentAngle[ch];
    currentAngle[ch] = (fabsf(diff) <= step) ? targetAngle[ch] : currentAngle[ch] + (diff > 0 ? step : -step);
    writeServo(ch, currentAngle[ch]);
  }
}

bool parseFloat(const String& s, float& out) {
  char* end;
  out = strtof(s.c_str(), &end);
  return end != s.c_str() && *end == '\0';
}

int splitTokens(const String& line, String* tokens, int maxTokens) {
  int n = 0, i = 0, len = line.length();
  while (i < len && n < maxTokens) {
    while (i < len && isspace(line[i])) i++;
    if (i >= len) break;
    int start = i;
    while (i < len && !isspace(line[i])) i++;
    tokens[n++] = line.substring(start, i);
  }
  return n;
}

void handleMove(const String* t, int n, bool smooth) {
  const char* name = smooth ? "Y" : "X";
  int nargs = n - 1;
  if (nargs != 2 && nargs != NUM_SERVOS) {
    out->println("Error: expected 2 or 16 arguments");
    return;
  }
  float v[NUM_SERVOS];
  for (int i = 0; i < nargs; i++) {
    if (!parseFloat(t[i + 1], v[i])) {
      out->println("Error: invalid number");
      return;
    }
  }
  if (nargs == 2) {
    int ch = (int)v[0];
    if (ch < 0 || ch >= NUM_SERVOS) {
      out->println("Error: channel out of range 0-15");
      return;
    }
    setTarget(ch, v[1], smooth);
    out->printf("OK %s %d %.2f\n", name, ch, clampAngle(v[1]));
  } else {
    for (int ch = 0; ch < NUM_SERVOS; ch++) setTarget(ch, v[ch], smooth);
    out->printf("OK %s all\n", name);
  }
}

void handleLine(const String& line) {
  String t[NUM_SERVOS + 2];
  int n = splitTokens(line, t, NUM_SERVOS + 2);
  if (n == 0) return;
  String cmd = t[0];
  cmd.toUpperCase();

  if (cmd == "X") handleMove(t, n, false);
  else if (cmd == "Y") handleMove(t, n, true);
  else if (cmd == "INFO") {
    out->printf("OK ErisServoWiFi-esp32 pca9685@0x%02X ap=%s ip=%s stations=%d port=%u cmds=X,Y,ON,OFF\n",
                PCA9685_ADDR, AP_SSID, WiFi.softAPIP().toString().c_str(),
                WiFi.softAPgetStationNum(), TCP_PORT);
  } else if (cmd == "ON") { digitalWrite(LED_PIN, HIGH); out->println("OK ON"); }
  else if (cmd == "OFF") { digitalWrite(LED_PIN, LOW); out->println("OK OFF"); }
  else out->println("Error: unknown command");
}

// Lee bytes de un stream y despacha lineas completas con 'out' apuntando a 'reply'.
void pump(Stream& in, Print& reply, String& buf) {
  while (in.available() > 0) {
    char c = in.read();
    if (c == '\n') {
      out = &reply;
      handleLine(buf);
      buf = "";
    } else if (c != '\r' && buf.length() < MAX_LINE) {
      buf += c;
    }
  }
}

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(PCA_SDA, PCA_SCL);
  Wire.setClock(400000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS[0] ? AP_PASS : nullptr, AP_CHANNEL, 0, 2);
  WiFi.setSleep(false);  // sin modem-sleep: baja la latencia de los comandos
  server.begin();
  server.setNoDelay(true);

  Serial.printf("READY servo_wifi ap=%s ssid=%s ip=%s port=%u\n", apOk ? "ok" : "FAIL", AP_SSID,
                WiFi.softAPIP().toString().c_str(), TCP_PORT);
}

void loop() {

  // Acepta un cliente nuevo (reemplaza al anterior si ya habia uno).
  WiFiClient incoming = server.available();
  if (incoming) {
    if (client) client.stop();
    client = incoming;
    client.setNoDelay(true);
    tcpLine = "";
  }

  pump(Serial, Serial, serialLine);
  if (client && client.connected()) pump(client, client, tcpLine);

  updateSmooth();
}
