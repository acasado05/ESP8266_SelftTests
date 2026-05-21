#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Sensores
Adafruit_ADS1115 ads;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

/* Configurar pines I2C */
const int sdaPin = 8;
const int sclPin = 9;

struct MedidasAmbientales {
  float tempAHT;
  float humAHT;
  float tempBMP;
  float tempAmbFinal;
};

struct DatosFotovoltaicos {
  float V_shunt_ESP32;
  float V_shunt_ADS;
  float Isc;  
  float G;
  float Tc_NOCT;
};

const int MUESTRAS_PROMEDIO = 20;

// Pin ADC interno ESP32-S3
const int internalAdcPin = 1; 
bool mostrar = false;

// Constantes
constexpr int NUM_MUESTRAS_ADC = 50;
constexpr float ADC_RES = 2047.0; 
constexpr int VOLTAGE_SCALE = 10; 
constexpr float DIV_TENS = 0.395; 
constexpr float RSHUNT = 0.1563 / 2.5;
constexpr float Isc_cal = 2.85; 
constexpr float G_cem = 1000.0;
constexpr float Voc_cal = 22.341; 
constexpr float beta = -0.00287;
constexpr float alfa = 0.00165; 
constexpr float T_cem = 25.0;
constexpr float NOCT = 0.031;

MedidasAmbientales realizarMedida (void);
DatosFotovoltaicos calcularParametrosSolares(float ambTemp);
void logDatosSerial(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv);

void setup() {
    Serial.begin(115200);
    
    unsigned long start = millis();
    while (!Serial && (millis() - start < 4000)) {
        delay(10);
    }

    delay(2000);

    // Inicializar I2C (ajusta pines si usas otros distintos al estándar)
    Wire.begin(sdaPin, sclPin); // SDA, SCL en ESP32-S3 (ejemplo común)

    // GAIN_ONE: Rango +/- 4.096V (1 bit = 0.125mV)
    ads.setGain(GAIN_ONE); 
    if (!ads.begin()) {
        Serial.println("Fallo al iniciar el ADS1115. Revisa conexiones.");
        while (1);
    }

    if(!aht.begin()) {
        Serial.println("Error: No se encontró AHT20");
    }

    if(!bmp.begin(BMP280_ADDRESS)) {
        Serial.println("Error: No se encontró BMP280");
    }

    analogSetAttenuation(ADC_11db);
}

void loop() {

    // 1. Obtener medidas ambientales
    MedidasAmbientales misMedidasAmb = realizarMedida();

    // 2. Obtener medidas fotovoltaicas (usando la temp ambiente calculada)
    DatosFotovoltaicos misDatosFV = calcularParametrosSolares(misMedidasAmb.tempAmbFinal);

    /// 3. Enviar todo al monitor serie
    logDatosSerial(misMedidasAmb, misDatosFV);

    delay(10000); // Una lectura por 10 segundos para comparar con calma
}

MedidasAmbientales realizarMedida (void){

  sensors_event_t h, t;
  float sumaTempAHT = 0.0f; float sumaHumAHT  = 0.0f; float sumaTempBMP = 0.0f;

  MedidasAmbientales medidas;

  for(int i = 0; i < MUESTRAS_PROMEDIO; i++){
    aht.getEvent(&h, &t);
    sumaTempAHT += t.temperature;
    sumaHumAHT += h.relative_humidity;
    sumaTempBMP += bmp.readTemperature();
    delay(50);
  }

  medidas.tempAHT = sumaTempAHT / MUESTRAS_PROMEDIO;
  medidas.humAHT = sumaHumAHT / MUESTRAS_PROMEDIO;
  medidas.tempBMP = sumaTempBMP / MUESTRAS_PROMEDIO;
  medidas.tempAmbFinal = (medidas.tempAHT + medidas.tempBMP) / 2.0;
  
  return medidas;
}

void logDatosSerial(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv) {
    Serial.println("\n--- NUEVA LECTURA DEL SISTEMA ---");
    
    Serial.println("[Datos Ambientales]");
    Serial.print("Temp BMP280              : "); Serial.print(amb.tempBMP, 2); Serial.println(" °C"); 
    Serial.print("Temp AHT20               : "); Serial.print(amb.tempAHT, 2); Serial.println(" °C");
    Serial.print("Temp Ambiente (Promedio) : "); Serial.print(amb.tempAmbFinal, 2); Serial.println(" °C");
    Serial.print("Humedad Relativa (AHT20) : "); Serial.print(amb.humAHT, 2); Serial.println(" %");
    
    Serial.println("\n[Comparativa V_Shunt]");
    Serial.print("ESP32-S3 (ADC Interno)   : "); Serial.print(fv.V_shunt_ESP32, 4); Serial.println(" V");
    Serial.print("ADS1115 (ADC Externo)    : "); Serial.print(fv.V_shunt_ADS, 4); Serial.println(" V");
    
    // Opcional: Mostrar el error absoluto
    float error = abs(fv.V_shunt_ESP32 - fv.V_shunt_ADS);
    Serial.print("Diferencia de lectura    : "); Serial.print(error, 4); Serial.println(" V");

    Serial.println("\n[Datos Fotovoltaicos (Basados en ADS1115)]");
    Serial.print("Corriente Isc            : "); Serial.print(fv.Isc, 3); Serial.println(" A");
    Serial.print("Irradiancia (G)          : "); Serial.print(fv.G, 2); Serial.println(" W/m2");
    Serial.print("Temp Célula (Tc_NOCT)    : "); Serial.print(fv.Tc_NOCT, 2); Serial.println(" °C");
    Serial.println("---------------------------------");
}

DatosFotovoltaicos calcularParametrosSolares(float ambTemp) {
    DatosFotovoltaicos datos;
    long sum_esp32_mV = 0;
    float sum_ads_V = 0;

    // 1. Toma de muestras promediada de la MISMA señal
    for (int i = 0; i < NUM_MUESTRAS_ADC; i++) {
        sum_esp32_mV += analogReadMilliVolts(internalAdcPin);
        
        int16_t results = ads.readADC_SingleEnded(0);
        sum_ads_V += ads.computeVolts(results); 
        
        delay(2); 
    }

    // 2. Promedios en Voltios
    datos.V_shunt_ESP32 = ((float)sum_esp32_mV / NUM_MUESTRAS_ADC) / 1000.0;
    datos.V_shunt_ADS   = sum_ads_V / NUM_MUESTRAS_ADC;

    // 3. Cálculos Fotovoltaicos (Usando el ADS1115 por su precisión)
    datos.Isc = datos.V_shunt_ADS / RSHUNT;
    
    // Irradiancia (aproximación directa sin realimentación térmica)
    datos.G = (datos.Isc * G_cem) / Isc_cal;
    
    // Temperatura de la célula (Modelo NOCT simplificado que tenías)
    datos.Tc_NOCT = ambTemp + (NOCT * datos.G); 

    return datos;
}