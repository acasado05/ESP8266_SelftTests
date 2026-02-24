#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

void setup() {
  delay(5000);
  Serial.begin(74880);
  Wire.begin(12, 14); // SDA=D6, SCL=D5
  
  aht.begin();
  bmp.begin(0x77); // O 0x77 según lo que descubriste
  Serial.println("Captura lista. Dale a RUN en PulseView...");
}

void loop() {
  sensors_event_t humidity, temp;
  
  // Ráfaga 1: AHT20
  aht.getEvent(&humidity, &temp); 
  
  // Ráfaga 2: BMP280
  float t = bmp.readTemperature();

  float tempTotal = (temp.temperature + t) / 2; // Suma las temperaturas para ver la diferencia
  
  Serial.printf("T: %.2f | H: %.2f | T_AHT: %.2f | T_BME: %.2f\n", tempTotal, humidity.relative_humidity, temp.temperature, t);
  delay(5000); // Pausa larga para ver los bloques separados
}