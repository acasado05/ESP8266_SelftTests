#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BMP280.h>

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
Adafruit_INA219 ina219(0x41);

float leerBusVoltageManual();

void setup() {
  Serial.begin(74880);
  while(!Serial); // Espera a que el monitor serial esté listo
  Wire.begin(12, 14); // SDA=D6, SCL=D5
  
  /*aht.begin();
  bmp.begin(0x77); // O 0x77 según lo que descubriste
  Serial.println("Captura lista. Dale a RUN en PulseView...");*/
  if(!ina219.begin()) {
    Serial.println("No se encontró el INA219. Verifica las conexiones.");
    while(1) {delay (10);};
  }

  ina219.setCalibration_32V_2A(); // Configura para medir hasta 32V y 2A
  //ina219.setCalibration_16V_400mA(); // Configura para medir hasta 16V y 400mA, con mayor precisión
  Serial.println("INA219 detectado y listo.");
}

void loop() {
  /*sensors_event_t humidity, temp;
  
  // Ráfaga 1: AHT20
  aht.getEvent(&humidity, &temp); 
  
  // Ráfaga 2: BMP280
  float t = bmp.readTemperature();

  float tempTotal = (temp.temperature + t) / 2; // Suma las temperaturas para ver la diferencia
  
  Serial.printf("T: %.2f | H: %.2f | T_AHT: %.2f | T_BME: %.2f\n", tempTotal, humidity.relative_humidity, temp.temperature, t);*/

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  // Lectura de parámetros
  busvoltage = leerBusVoltageManual(); // Usamos la lectura manual para comparar con la librería
  shuntvoltage = ina219.getShuntVoltage_mV();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();

  if(current_mA == 0 && shuntvoltage > 0) {
    current_mA = shuntvoltage / 0.1; // Asumiendo Rshunt de 0.1 ohmios
  }
  
  // Salida formateada similar a tu ejemplo anterior
  Serial.printf("V_Bus: %.2f V | I: %.2f mA | P: %.2f mW | V_Shunt: %.2f mV\n", busvoltage, current_mA, power_mW, shuntvoltage);

  delay(5000); // Pausa larga para ver los bloques separados
}

float leerBusVoltageManual() {
  uint16_t value;
  Wire.beginTransmission(0x41);
  Wire.write(0x02); 
  Wire.endTransmission();
  
  Wire.requestFrom(0x41, (uint8_t)2);
  if (Wire.available() == 2) {
    value = (Wire.read() << 8) | Wire.read();
  }
  
  // 1. Extraemos el dato desplazando 3 bits
  float v_crudo = (float)((value >> 3) * 4) / 1000.0;
  
  // 2. Aplicamos el factor de corrección K para ajustar a tus 6V reales
  // Si tu multímetro dice que la fuente da 6.0V y el código dice 7.0V:
  float K = 0.857; 
  float tension_final = v_crudo * K;
  
  return tension_final;
}