/*
 * ESP32: 16 servos por PCA9685 + INA219, protocolo compatible con Eris ErisServoDriver.
 *
 * Reimplementacion autonoma (no usa la libreria eriscommon) del protocolo de
 * texto del flavor ErisServoDriver, mas lectura de corriente con un INA219.
 *
 * Librerias (Library Manager): "Adafruit PWM Servo Driver Library",
 * "INA219" de Rob Tillaart. Board: ESP32 Dev Module. Baud: 921600.
 *
 * Cableado:
 *   PCA9685 -> bus I2C 0 (Wire):  SDA=GPIO21, SCL=GPIO22, direccion 0x40
 *   INA219  -> bus I2C 1 (Wire1): SDA=GPIO32, SCL=GPIO33, direccion 0x40
 * Son pines dedicados del ESP32, cableados directo al modulo INA219 (no
 * pasan por el PCA9685). Al ser un bus I2C distinto del PCA9685, ambos
 * pueden usar la direccion 0x40 sin conflicto.
 *
 * Calibracion: INA219_MAX_CURRENT_A / INA219_SHUNT_OHMS deben coincidir con
 * el modulo real (la mayoria de los breakouts INA219 traen un shunt de
 * 0.1 ohm). Ajustar INA219_MAX_CURRENT_A a la corriente pico esperada por
 * servo para tener mejor resolucion.
 *
 * Protocolo: lineas de texto terminadas en '\n', tokens separados por espacio.
 *   X <ch> <ang>            mueve un canal (0-15) al angulo (0-180), inmediato
 *   X <a0> ... <a15>        mueve los 16 canales, inmediato
 *   Y <ch> <ang> | Y <a0>..<a15>   igual pero con rampa (SMOOTH_SPEED_DPS)
 *   INFO                    info del firmware
 *   ON / OFF                LED de la placa
 *   INA                     una lectura: "D <t_ms> <mA> <V_bus> <mW>"
 *   S_F INA                 selecciona la feature INA para streaming
 *   S_TIME                  pone t0 = ahora (timestamps parten de 0)
 *   S_ON / S_OFF            inicia/detiene streaming de lineas "D ..." (INA_PERIOD_MS)
 *   MON ON / MON OFF        modo monitor: imprime lecturas del INA219 en texto
 *                           legible cada MONITOR_PERIOD_MS, para usar a mano
 *                           desde el Monitor Serie del IDE (no requiere S_F/S_TIME/S_ON)
 *   SCAN                    escanea ambos buses I2C (Wire y Wire1) y lista las
 *                           direcciones encontradas; util si aparece "INA not found"
 * Cada comando responde exactamente una linea ("OK ..." o "Error: ..."), salvo
 * las lineas "D ..." de S_ON y "INA ..." de MON ON, que se imprimen solas y
 * pueden intercalarse con las respuestas de otros comandos (p.ej. X/Y).
 *
 * Uso tipico desde el Monitor Serie (terminador de linea = "Newline", 921600 baud):
 *   MON ON       -> empieza a imprimir "INA t=.. I=.. V=.. P=.." cada segundo
 *   X 0 90       -> mueve el servo del canal 0 a 90 grados (respuesta "OK X 0 90.00")
 *   MON OFF      -> detiene la impresion periodica
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <INA219.h>

const unsigned long SERIAL_BAUD = 921600;

const int PCA_SDA = 21, PCA_SCL = 22;
const uint8_t PCA9685_ADDR = 0x40;

const int INA_SDA = 32, INA_SCL = 33;
const uint8_t INA219_ADDR = 0x41;  // confirmado con "SCAN": el modulo responde en 0x41, no 0x40
const unsigned long INA_PERIOD_MS = 10;  // 100 Hz (streaming "D ..." para el driver Python)
const unsigned long MONITOR_PERIOD_MS = 1000;  // 1 Hz (modo "MON ON" legible en Monitor Serie)
const float INA219_MAX_CURRENT_A = 2.0;   // ajustar a la corriente pico esperada por servo
const float INA219_SHUNT_OHMS = 0.1;      // shunt del modulo (ver serigrafia, tipico 0.1 ohm)

const int LED_PIN = 2;

const int NUM_SERVOS = 16;
const float ANGLE_MIN = 0, ANGLE_MAX = 180;
const int SERVO_MIN_US = 500, SERVO_MAX_US = 2400;
const float SMOOTH_SPEED_DPS = 90.0;  // velocidad de la rampa de Y (grados/s)
const unsigned long SERVO_UPDATE_MS = 20;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR, Wire);
INA219 ina(INA219_ADDR, &Wire1);
bool inaOk = false;

float currentAngle[NUM_SERVOS];
float targetAngle[NUM_SERVOS];
bool known[NUM_SERVOS];  // false hasta el primer comando: evita saltos al arrancar

bool streaming = false;
bool inaSelected = false;
bool monitorMode = false;
uint32_t t0Us = 0;
unsigned long lastServoMs = 0, lastInaMs = 0, lastMonMs = 0, lastCalMs = 0;
const unsigned long CAL_PERIOD_MS = 2000;  // re-arma la calibracion del INA219 por si un sag de
                                            // tension la borra (el chip pierde el registro de
                                            // calibracion en un POR pero sigue midiendo bus_V)

String inputLine;
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

void printSample() {
  Serial.printf("D %.3f %.3f %.3f %.3f\n", (uint32_t)(micros() - t0Us) / 1000.0,
                ina.getCurrent_mA(), ina.getBusVoltage(), ina.getPower_mW());
}

void printMonitorSample() {
  Serial.printf("INA t=%.1fs I=%.1fmA V=%.3fV P=%.1fmW\n", (micros() - t0Us) / 1000000.0,
                ina.getCurrent_mA(), ina.getBusVoltage(), ina.getPower_mW());
}

void scanBus(TwoWire& w, const char* name) {
  int found = 0;
  Serial.printf("Scan %s:", name);
  for (uint8_t addr = 1; addr < 127; addr++) {
    w.beginTransmission(addr);
    if (w.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if (found == 0) Serial.print(" (nada)");
  Serial.println();
}

void handleMove(const String* t, int n, bool smooth) {
  const char* name = smooth ? "Y" : "X";
  int nargs = n - 1;
  if (nargs != 2 && nargs != NUM_SERVOS) {
    Serial.println("Error: expected 2 or 16 arguments");
    return;
  }
  float v[NUM_SERVOS];
  for (int i = 0; i < nargs; i++) {
    if (!parseFloat(t[i + 1], v[i])) {
      Serial.println("Error: invalid number");
      return;
    }
  }
  if (nargs == 2) {
    int ch = (int)v[0];
    if (ch < 0 || ch >= NUM_SERVOS) {
      Serial.println("Error: channel out of range 0-15");
      return;
    }
    setTarget(ch, v[1], smooth);
    Serial.printf("OK %s %d %.2f\n", name, ch, clampAngle(v[1]));
  } else {
    for (int ch = 0; ch < NUM_SERVOS; ch++) setTarget(ch, v[ch], smooth);
    Serial.printf("OK %s all\n", name);
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
    Serial.printf("OK ErisServoINA-esp32 pca9685@0x%02X ina219@0x%02X(Wire1) ina=%s baud=%lu cmds=X,Y,MON,INA,SCAN,S_F,S_TIME,S_ON,S_OFF,ON,OFF\n",
                  PCA9685_ADDR, INA219_ADDR, inaOk ? "ok" : "NOT_FOUND", SERIAL_BAUD);
  } else if (cmd == "ON") { digitalWrite(LED_PIN, HIGH); Serial.println("OK ON"); }
  else if (cmd == "OFF") { digitalWrite(LED_PIN, LOW); Serial.println("OK OFF"); }
  else if (cmd == "INA") {
    if (inaOk) printSample(); else Serial.println("Error: INA219 not found");
  } else if (cmd == "S_F") {
    inaSelected = false;
    for (int i = 1; i < n; i++) {
      String f = t[i];
      f.toUpperCase();
      if (f == "INA") inaSelected = true;
      else { Serial.println("Error: unknown feature (only INA)"); return; }
    }
    Serial.println("OK S_F");
  } else if (cmd == "S_TIME") { t0Us = micros(); Serial.println("OK S_TIME"); }
  else if (cmd == "S_ON") {
    if (!inaOk) Serial.println("Error: INA219 not found");
    else if (!inaSelected) Serial.println("Error: no feature selected (S_F INA)");
    else { streaming = true; lastInaMs = millis(); Serial.println("OK S_ON"); }
  } else if (cmd == "S_OFF") { streaming = false; Serial.println("OK S_OFF"); }
  else if (cmd == "MON") {
    if (n != 2) { Serial.println("Error: expected MON ON|OFF"); return; }
    String arg = t[1];
    arg.toUpperCase();
    if (arg == "ON") {
      if (!inaOk) { Serial.println("Error: INA219 not found"); return; }
      monitorMode = true;
      lastMonMs = millis();
      Serial.println("OK MON ON");
    } else if (arg == "OFF") {
      monitorMode = false;
      Serial.println("OK MON OFF");
    } else {
      Serial.println("Error: expected MON ON|OFF");
    }
  } else if (cmd == "SCAN") {
    scanBus(Wire, "Wire(pca9685,21/22)");
    scanBus(Wire1, "Wire1(ina219,32/33)");
    Serial.println("OK SCAN");
  } else Serial.println("Error: unknown command");
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

  Wire1.begin(INA_SDA, INA_SCL);
  Wire1.setClock(400000);
  inaOk = ina.begin() && ina.setMaxCurrentShunt(INA219_MAX_CURRENT_A, INA219_SHUNT_OHMS);

  t0Us = micros();
  Serial.printf("READY servo_ina ina=%s\n", inaOk ? "ok" : "NOT_FOUND");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handleLine(inputLine);
      inputLine = "";
    } else if (c != '\r' && inputLine.length() < MAX_LINE) {
      inputLine += c;
    }
  }

  updateSmooth();

  if (inaOk && millis() - lastCalMs >= CAL_PERIOD_MS) {
    lastCalMs = millis();
    ina.setMaxCurrentShunt(INA219_MAX_CURRENT_A, INA219_SHUNT_OHMS);
  }

  if (streaming && millis() - lastInaMs >= INA_PERIOD_MS) {
    lastInaMs += INA_PERIOD_MS;
    printSample();
  }

  if (monitorMode && millis() - lastMonMs >= MONITOR_PERIOD_MS) {
    lastMonMs += MONITOR_PERIOD_MS;
    printMonitorSample();
  }
}
