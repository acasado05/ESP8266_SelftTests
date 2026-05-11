#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Configuración ADS1115
Adafruit_ADS1115 ads;

/* Configurar pines I2C */
const int sdaPin = 8;
const int sclPin = 9;

// Pin ADC interno ESP32-S3
const int internalAdcPin = 1; 

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Inicializar I2C (ajusta pines si usas otros distintos al estándar)
    Wire.begin(sdaPin, sclPin); // SDA, SCL en ESP32-S3 (ejemplo común)

    // Configurar ADS1115
    // GAIN_ONE: Rango +/- 4.096V (1 bit = 0.125mV)
    ads.setGain(GAIN_ONE); 
    if (!ads.begin()) {
        Serial.println("Fallo al iniciar el ADS1115. Revisa conexiones.");
        while (1);
    }

    Serial.println("--- COMPARATIVA DE ADCs ---");
    Serial.println("V_Fuente (Multímetro) | V_ESP32_S3 (mV) | V_ADS1115 (mV)");
}

void loop() {
    // 1. Lectura ADC Interno (usando calibración de fábrica)
    // analogReadMilliVolts devuelve directamente el valor en mV
    uint32_t v_internal = analogReadMilliVolts(internalAdcPin);

    // 2. Lectura ADS1115
    int16_t results = ads.readADC_SingleEnded(0);
    float v_ads = ads.computeVolts(results) * 1000; // Convertir a mV

    // 3. Imprimir resultados
    Serial.print("LECTURA: ");
    Serial.print(v_internal);
    Serial.print(" mV (ESP32) | ");
    Serial.print(v_ads, 2);
    Serial.println(" mV (ADS1115)");

    delay(1000); // Una lectura por segundo para comparar con calma
}