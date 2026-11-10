#include <Wire.h>
#include <Adafruit_INA219.h>

// ESP32: SDA en GPIO32 y SCL en GPIO33.
// Instalar la biblioteca Adafruit INA219 y sus dependencias.
constexpr int SDA_PIN = 32;
constexpr int SCL_PIN = 33;

Adafruit_INA219 ina219(0x40);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // El orden de los pines es SDA, SCL.
  Wire.begin(SDA_PIN, SCL_PIN, 100000);

  Serial.println("Buscando INA219...");

  if (!ina219.begin(&Wire)) {
    Serial.println("No se encontro el INA219 en 0x40.");
    Serial.println("Revisa alimentacion, GND, SDA, SCL y direccion I2C.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("INA219 detectado correctamente.");
}

void loop() {
  float voltajeBus = ina219.getBusVoltage_V();
  float voltajeShunt = ina219.getShuntVoltage_mV();
  float corriente = ina219.getCurrent_mA();
  float potencia = ina219.getPower_mW();

  Serial.println("---------- INA219 ----------");
  Serial.printf("Voltaje bus (VIN-): %.3f V\n", voltajeBus);
  Serial.printf("Voltaje shunt:     %.3f mV\n", voltajeShunt);
  Serial.printf("Corriente:         %.3f mA\n", corriente);
  Serial.printf("Potencia:          %.3f mW\n\n", potencia);

  delay(1000);
}
