/*
 * Control de 3 servos por ESP32 via Serial (USB).
 *
 * Pines: 13, 26, 15
 *
 * Instalar en Arduino IDE: Tools > Manage Libraries > "ESP32Servo" (Kevin Harrington).
 * Board: ESP32 DevKit genérico. Baudrate: 115200.
 *
 * Protocolo (una linea de texto terminada en '\n'):
 *   <angulo_pin13>,<angulo_pin26>,<angulo_pin15>[,<duracion_ms>]
 * Los tres angulos son obligatorios y se aplican en ese orden fijo. El
 * cuarto campo (opcional) es la duracion en ms de la rampa hacia ese
 * objetivo; si se omite, se usa DEFAULT_MOVE_DURATION_MS (3 seg).
 *
 * Ejemplos:
 *   0,0,0            -> rampa de 3000 ms (default), para comandos manuales
 *   45.00,80.00,10.00,80  -> rampa de 80 ms, pensado para reproducir una
 *                            trayectoria punto a punto sin que cada paso
 *                            reinicie una rampa larga de 3 seg
 *
 * El movimiento NO es inmediato: cada servo se desplaza gradualmente desde
 * su posicion actual hasta la posicion pedida, en la duracion indicada. Un
 * comando nuevo mientras otro esta en curso reemplaza el movimiento,
 * tomando como punto de partida la posicion actual interpolada.
 *
 * Respuesta por Serial al aceptar el comando (no espera a que termine el
 * movimiento):
 *   OK 13:90.00 26:45.00 15:120.00
 * o, si la linea no pudo parsearse:
 *   ERR bad_format
 */

#include <ESP32Servo.h>

const int NUM_SERVOS = 3;
const int SERVO_PINS[NUM_SERVOS] = {13, 26, 15};

Servo servos[NUM_SERVOS];
// Posicion inicial de armado: knee (pin 26, indice 1) arranca en 180 grados,
// hip y ankle en 0. Sin inversion, el angulo pedido es siempre el fisico.
float currentAngle[NUM_SERVOS] = {0, 180, 0};  // posicion interpolada, actualizada en cada paso
float startAngle[NUM_SERVOS] = {0, 180, 0};    // posicion al iniciar el movimiento en curso
float targetAngle[NUM_SERVOS] = {0, 180, 0};   // posicion pedida por el ultimo comando

const unsigned long SERIAL_BAUD = 115200;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

const unsigned long DEFAULT_MOVE_DURATION_MS = 3000;  // usado si el comando no trae duracion
const unsigned long MIN_MOVE_DURATION_MS = 20;
const unsigned long MAX_MOVE_DURATION_MS = 10000;
const unsigned long UPDATE_INTERVAL_MS = 20;  // ~50Hz, paso de interpolacion

unsigned long moveStartMs = 0;
unsigned long moveDurationMs = DEFAULT_MOVE_DURATION_MS;
unsigned long lastUpdateMs = 0;
bool moving = false;

String inputLine;

float clampAngle(float angle) {
  if (angle < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
  if (angle > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
  return angle;
}

void printStatus() {
  Serial.print("OK");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(" ");
    Serial.print(SERVO_PINS[i]);
    Serial.print(":");
    Serial.print(targetAngle[i], 2);
  }
  Serial.println();
}

bool handleLine(const String& line) {
  // Cuenta las comas para saber si trae 3 campos (angulos, duracion default)
  // o 4 (angulos + duracion en ms explicita).
  int len = line.length();
  int commaCount = 0;
  for (int i = 0; i < len; i++) {
    if (line[i] == ',') commaCount++;
  }

  bool hasDuration = (commaCount == NUM_SERVOS);
  if (commaCount != NUM_SERVOS - 1 && !hasDuration) return false;

  int fieldCount = NUM_SERVOS + (hasDuration ? 1 : 0);
  float parsed[NUM_SERVOS];
  unsigned long duration = DEFAULT_MOVE_DURATION_MS;
  int start = 0;

  for (int i = 0; i < fieldCount; i++) {
    int comma = line.indexOf(',', start);
    bool isLast = (i == fieldCount - 1);

    if (isLast != (comma == -1)) return false;

    String token = (comma == -1) ? line.substring(start) : line.substring(start, comma);
    token.trim();
    if (token.length() == 0) return false;

    if (i < NUM_SERVOS) {
      parsed[i] = clampAngle(token.toFloat());
    } else {
      long d = token.toInt();
      if (d < (long)MIN_MOVE_DURATION_MS) d = MIN_MOVE_DURATION_MS;
      if (d > (long)MAX_MOVE_DURATION_MS) d = MAX_MOVE_DURATION_MS;
      duration = (unsigned long)d;
    }

    start = comma + 1;
  }

  // Arranca un nuevo movimiento gradual desde la posicion actual (interpolada)
  // hacia el objetivo pedido; reemplaza cualquier movimiento en curso.
  for (int i = 0; i < NUM_SERVOS; i++) {
    startAngle[i] = currentAngle[i];
    targetAngle[i] = parsed[i];
  }
  moveStartMs = millis();
  moveDurationMs = duration;
  moving = true;
  return true;
}

void updateServos() {
  if (!moving) return;

  unsigned long now = millis();
  if (now - lastUpdateMs < UPDATE_INTERVAL_MS) return;
  lastUpdateMs = now;

  float t = (float)(now - moveStartMs) / (float)moveDurationMs;
  if (t >= 1.0) {
    t = 1.0;
    moving = false;
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = startAngle[i] + (targetAngle[i] - startAngle[i]) * t;
    servos[i].write(currentAngle[i]);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  for (int i = 0; i < NUM_SERVOS; i++) {
    ESP32PWM::allocateTimer(i);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].write(currentAngle[i]);
  }

  Serial.println("READY servo_control");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inputLine.trim();
      if (inputLine.length() > 0) {
        if (handleLine(inputLine)) {
          printStatus();
        } else {
          Serial.println("ERR bad_format");
        }
      }
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }

  updateServos();
}
