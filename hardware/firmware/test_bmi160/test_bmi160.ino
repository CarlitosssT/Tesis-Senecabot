#include <Wire.h>

// BMI160 en un bus I2C propio del ESP32: SDA=GPIO25, SCL=GPIO26.
// CS -> 3V3 (modo I2C), SAO -> GND (0x68) o 3V3 (0x69).
// No requiere bibliotecas externas: se leen los registros directamente.
constexpr int SDA_PIN = 25;
constexpr int SCL_PIN = 26;

constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_DATA = 0x0C;  // gyro XYZ (6 bytes) seguido de accel XYZ (6 bytes)
constexpr uint8_t REG_CMD = 0x7E;
constexpr uint8_t CHIP_ID = 0xD1;

// Rangos por defecto tras el reset: accel +-2 g, gyro +-2000 dps.
constexpr float ACC_LSB_PER_G = 16384.0f;
constexpr float GYR_LSB_PER_DPS = 16.4f;

uint8_t addr = 0;

bool readRegs(uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool probe(uint8_t a) {
  addr = a;
  uint8_t id;
  return readRegs(REG_CHIP_ID, &id, 1) && id == CHIP_ID;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN, 100000);

  Serial.println("Buscando BMI160...");
  if (!probe(0x68) && !probe(0x69)) {
    Serial.println("No se encontro el BMI160 en 0x68 ni 0x69.");
    Serial.println("Revisa VIN=3V3, GND, SDA=25, SCL=26 y CS=3V3.");
    while (true) delay(1000);
  }
  Serial.printf("BMI160 detectado en 0x%02X (CHIP_ID=0xD1).\n", addr);

  writeReg(REG_CMD, 0x11);  // acelerometro en modo normal
  delay(10);
  writeReg(REG_CMD, 0x15);  // giroscopio en modo normal
  delay(100);
}

void loop() {
  uint8_t b[12];
  if (!readRegs(REG_DATA, b, sizeof(b))) {
    Serial.println("Error de lectura I2C");
    delay(500);
    return;
  }

  int16_t gx = (int16_t)(b[1] << 8 | b[0]);
  int16_t gy = (int16_t)(b[3] << 8 | b[2]);
  int16_t gz = (int16_t)(b[5] << 8 | b[4]);
  int16_t ax = (int16_t)(b[7] << 8 | b[6]);
  int16_t ay = (int16_t)(b[9] << 8 | b[8]);
  int16_t az = (int16_t)(b[11] << 8 | b[10]);

  Serial.printf("Accel [g]:   X=%7.3f  Y=%7.3f  Z=%7.3f\n",
                ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G);
  Serial.printf("Gyro [dps]:  X=%7.2f  Y=%7.2f  Z=%7.2f\n\n",
                gx / GYR_LSB_PER_DPS, gy / GYR_LSB_PER_DPS, gz / GYR_LSB_PER_DPS);

  delay(200);
}
