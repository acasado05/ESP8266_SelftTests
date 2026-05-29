#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <time.h>

#define DEBUG_MODE false // true para ver tramas crudas Modbus

// --- Configuración hardware RS-485 (Inversor) ---
#define RS485_TX    17    // GPIO17 -> Módulo DI
#define RS485_RX    16    // GPIO16 <- Módulo RO
#define RE_DE_PIN   4     // GPIO4  -> Control RE_DE
#define HUAWEI_ID   1    // ID por defecto del Inversor
#define RS485_BAUD  9600 
#define TIMEOUT_MS  1500

// --- Registros Inversor Huawei ---
#define REG_PV2_VOLTAGE   32018   // U16, Gain 10 (V)
#define REG_PV2_CURRENT   32019   // I16, Gain 100 (A)
#define REG_P_INPUT_DC    32064   // I32, Gain 1 (W)
#define REG_P_ACTIVE_AC   32080   // I32, Gain 1 (W)

// --- Credenciales ---
const char* ssid = "TP-LINK_C062";
const char* password = "77817570";
const char* esiosToken = "76f5317763cf71beec91ede16a667d8428e1ff3b793d45665a3c803e5ea5e68e";

// --- Configuración NTP y Zona Horaria (Madrid) ---
const char* MY_TZ = "CET-1CEST,M3.5.0,M10.5.0/3"; 
const char* ntpServer = "pool.ntp.org";

// --- Variables ESIOS Temporizador (10 minutos) ---
unsigned long lastTimeRequest = 0;
const unsigned long timerDelay = 600000; 
float precioActualKWh = 0.0;

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

struct DatosInversor {
  float v_pv2;
  float i_pv2;
  int32_t p_dc_in;
  int32_t p_ac_out;
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

// --- Declaración de Funciones ---
void wifiSetUp();
void setNTP();
String getTimeStamp();
void obtenerPrecioESIOS();
MedidasAmbientales realizarMedida(void);
DatosFotovoltaicos calcularParametrosSolares(float ambTemp);
DatosInversor leerInversorHuawei();
void logDatosSerial(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv, const DatosInversor& inv);

// Funciones auxiliares Modbus
uint16_t crc16(const uint8_t *data, uint8_t len);
void sendModbusRequest(uint8_t slaveId, uint8_t funcCode, uint16_t regAddr, uint16_t numRegs);
uint8_t readModbusResponse(uint8_t *buf, uint8_t maxLen);
void printHex(const uint8_t *buf, uint8_t len);

void setup() {
  Serial.begin(115200);
    
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
      delay(10);
  }

  delay(2000);

  // 1. Configuración RS485 para Inversor
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Modo recepción inicial
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);

  wifiSetUp();

  // Inicializar I2C (ajusta pines si usas otros distintos al estándar)
  Wire.begin(sdaPin, sclPin); // SDA, SCL en ESP32-S3 (ejemplo común)

  // GAIN_ONE: Rango +/- 4.096V (1 bit = 0.125mV)
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
  ads.setGain(GAIN_EIGHT); // GAIN_EIGHT: Rango +/- 0.512V

  // La configuración NTP es bloqueante hasta que sincroniza, por eso va aqui al final
  setNTP();  // Configura el NTP para sincronizar la hora

  // Obtenego el precio nada más arrancar
  obtenerPrecioESIOS();
  lastTimeRequest = millis();
}

void loop() {

  unsigned long currentMillis = millis();

  // 1. Comprobar si han pasado 10 minutos para actualizar el precio
    if (currentMillis - lastTimeRequest >= timerDelay) {
        lastTimeRequest = currentMillis;
        obtenerPrecioESIOS();
    }

  // 2. Obtener medidas ambientales
  MedidasAmbientales misMedidasAmb = realizarMedida();

  // 3. Obtener medidas fotovoltaicas (usando la temp ambiente calculada)
  DatosFotovoltaicos misDatosFV = calcularParametrosSolares(misMedidasAmb.tempAmbFinal);

  // 4. Lecturas del Inversor Huawei
  DatosInversor misDatosInv = leerInversorHuawei();

  /// 5. Enviar todo al monitor serie
  logDatosSerial(misMedidasAmb, misDatosFV, misDatosInv);

  delay(60000); // Una lectura por 10 segundos para comparar con calma
}

// --------------------------------------------------------
// IMPLEMENTACIÓN DE FUNCIONES
// --------------------------------------------------------

DatosInversor leerInversorHuawei() {
  DatosInversor inv = {0.0, 0.0, 0, 0};
  uint8_t res[64];

  // 1. Leer String 2 (Tensión y Corriente) - 2 registros
  sendModbusRequest(HUAWEI_ID, 0x03, REG_PV2_VOLTAGE, 2);
  if (readModbusResponse(res, sizeof(res)) >= 7) {
    inv.v_pv2 = (float)((uint16_t)res[3] << 8 | res[4]) / 10.0f;
    inv.i_pv2 = (float)((int16_t)res[5] << 8 | res[6]) / 100.0f;
  }
  delay(100);

  // 2. Leer Potencia Entrada DC (I32)
  sendModbusRequest(HUAWEI_ID, 0x03, REG_P_INPUT_DC, 2);
  if (readModbusResponse(res, sizeof(res)) >= 9) {
    inv.p_dc_in = (int32_t)res[3] << 24 | (int32_t)res[4] << 16 | (int32_t)res[5] << 8 | (int32_t)res[6];
  }
  delay(100);

  // 3. Leer Potencia Salida AC (I32)
  sendModbusRequest(HUAWEI_ID, 0x03, REG_P_ACTIVE_AC, 2);
  if (readModbusResponse(res, sizeof(res)) >= 9) {
    inv.p_ac_out = (int32_t)res[3] << 24 | (int32_t)res[4] << 16 | (int32_t)res[5] << 8 | (int32_t)res[6];
  }

  return inv;
}

void obtenerPrecioESIOS() {
    if (WiFi.status() != WL_CONNECTED) return;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("[TIME] Error: No se pudo obtener la hora local para ESIOS");
        return;
    }

    char fechaHoy[11];
    strftime(fechaHoy, sizeof(fechaHoy), "%Y-%m-%d", &timeinfo);

    String urlDinamica = "https://api.esios.ree.es/indicators/1001?start_date=";
    urlDinamica += String(fechaHoy) + "T00:00:00Z&end_date=";
    urlDinamica += String(fechaHoy) + "T23:59:59Z&geo_ids[]=8741";

    Serial.print("\n[ESIOS] Consultando URL... ");

    WiFiClientSecure *client = new WiFiClientSecure;
    client->setInsecure(); 

    HTTPClient http;
    http.begin(*client, urlDinamica); 
    
    http.addHeader("x-api-key", esiosToken); 
    http.addHeader("Accept", "application/json; application/vnd.esios-api.v1+json");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("User-Agent", "ESP32-S3-Client");

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
        JsonDocument filter;
        filter["indicator"]["values"][0]["value"] = true;
        filter["indicator"]["values"][0]["datetime"] = true;

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));

        if (!error) {
            JsonArray values = doc["indicator"]["values"];
            char patronHora[10];
            snprintf(patronHora, sizeof(patronHora), "T%02d:00:00", timeinfo.tm_hour);
            bool encontrado = false;

            for (JsonObject v : values) {
                const char* datetimeStr = v["datetime"];
                if (datetimeStr != NULL && strstr(datetimeStr, patronHora) != NULL) {
                    float precioMWh = v["value"];
                    precioActualKWh = precioMWh / 1000.0;
                    encontrado = true;
                    break; 
                }
            }
            
            if (encontrado) {
                Serial.printf("[OK] Precio actualizado: %.4f €/kWh\n", precioActualKWh);
            } else {
                Serial.printf("[ESIOS] Tramo '%s' no disponible en el JSON todavía.\n", patronHora);
            }

        } else {
            Serial.printf("[JSON] Error de parseo: %s\n", error.c_str());
        }
    } else {
        Serial.printf("[HTTP] Error ESIOS: %d\n", httpResponseCode);
    }

    http.end();
    delete client; 
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

DatosFotovoltaicos calcularParametrosSolares(float ambTemp) {
    DatosFotovoltaicos datos;
    long sum_esp32_mV = 0;
    float sum_ads_mV = 0;

    // 1. Toma de muestras promediada de la MISMA señal
    for (int i = 0; i < NUM_MUESTRAS_ADC; i++) {
        sum_esp32_mV += analogReadMilliVolts(internalAdcPin);
        
        //int16_t results = ads.readADC_SingleEnded(0);
        int16_t results = ads.readADC_Differential_0_1();

        sum_ads_mV += ads.computeVolts(results) * 1000.0f;
        
        delay(25); 
    }

    float avg_esp32_mv = (float)sum_esp32_mV / NUM_MUESTRAS_ADC;
    float avg_ads_mv   = sum_ads_mV / NUM_MUESTRAS_ADC;

    // 2. Promedios en Voltios
    datos.V_shunt_ESP32 = avg_esp32_mv / 1000.0f;
    datos.V_shunt_ADS   = avg_ads_mv / 1000.0f;

    // 3. Cálculos Fotovoltaicos (Usando el ADS1115 por su precisión)
    datos.Isc = datos.V_shunt_ADS / RSHUNT;
    
    // Irradiancia (aproximación directa sin realimentación térmica)
    datos.G = (datos.Isc * G_cem) / Isc_cal;
    
    // Temperatura de la célula (Modelo NOCT simplificado que tenías)
    datos.Tc_NOCT = ambTemp + (NOCT * datos.G); 

    return datos;
}

void wifiSetUp() {
  Serial.print("\nConectando a: ");
  Serial.println(ssid);
  WiFi.hostname("ESP32_t0rt1s");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WIFI] ¡Conectado!");
}

String getTimeStamp(){
  time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);
  char timestamp[30];

  if(p_tm->tm_year > 70){ // Verifica que el año sea posterior a 1970
    sprintf(timestamp, "%02d/%02d/%04d %02d:%02d:%02d", 
            p_tm->tm_mday, 
            p_tm->tm_mon + 1, 
            p_tm->tm_year + 1900,
            p_tm->tm_hour, 
            p_tm->tm_min, 
            p_tm->tm_sec);

    return String(timestamp);
  } else {
    return String("Error conexión NTP");
  }
}

void setNTP() {
  Serial.println("[NTP] Sincronizando hora...");
  configTzTime(MY_TZ, ntpServer, "time.google.com");
  
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("\n[NTP] Reloj sincronizado con éxito.");
}

// =============================================================================
// FUNCIONES AUXILIARES RS485 / MODBUS
// =============================================================================
void sendModbusRequest(uint8_t slaveId, uint8_t funcCode, uint16_t regAddr, uint16_t numRegs) {
  while (Serial2.available()) Serial2.read(); // Limpiar buffer

  uint8_t frame[8];
  frame[0] = slaveId;
  frame[1] = funcCode;
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] =  regAddr       & 0xFF;
  frame[4] = (numRegs >> 8) & 0xFF;
  frame[5] =  numRegs       & 0xFF;

  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  digitalWrite(RE_DE_PIN, HIGH); // Activar TX
  delayMicroseconds(50); 
  Serial2.write(frame, 8);
  Serial2.flush();
  delayMicroseconds(50);
  digitalWrite(RE_DE_PIN, LOW);  // Volver a RX
}

uint8_t readModbusResponse(uint8_t *buf, uint8_t maxLen) {
  uint32_t t0 = millis();
  uint8_t idx = 0;
  
  while (!Serial2.available()) {
    if (millis() - t0 > TIMEOUT_MS) return 0;
  }

  t0 = millis();
  while (millis() - t0 < TIMEOUT_MS) {
    if (Serial2.available()) {
      uint8_t c = Serial2.read();
      if (idx == 0 && c == 0x00) continue; // Filtro de ruido
      if (idx < maxLen) buf[idx++] = c;
      t0 = millis(); 
    }
    if (idx > 0 && !Serial2.available() && (millis() - t0 > 4)) break;
  }

  if (DEBUG_MODE && idx > 0) {
    Serial.print("  [RX-Limpia] ");
    printHex(buf, idx);
    Serial.printf(" (%d bytes)\n", idx);
  }
  return idx;
}

uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

void printHex(const uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
}

void logDatosSerial(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv, const DatosInversor& inv) {
    Serial.println("\n=============================================");
    Serial.print(" TIMESTAMP: "); 
    Serial.println(getTimeStamp());
    Serial.println("=============================================");
    
    Serial.println("[Datos Ambientales]");
    Serial.print("Temp BMP280              : "); Serial.print(amb.tempBMP, 2); Serial.println(" °C"); 
    Serial.print("Temp AHT20               : "); Serial.print(amb.tempAHT, 2); Serial.println(" °C");
    Serial.print("Temp Ambiente (Promedio) : "); Serial.print(amb.tempAmbFinal, 2); Serial.println(" °C");
    Serial.print("Humedad Relativa (AHT20) : "); Serial.print(amb.humAHT, 2); Serial.println(" %");
    
    Serial.println("\n[Comparativa V_Shunt]");
    Serial.print("ESP32-S3 (ADC Interno)   : "); Serial.print(fv.V_shunt_ESP32, 4); Serial.println(" V");
    Serial.print("ADS1115 (ADC Externo)    : "); Serial.print(fv.V_shunt_ADS, 4); Serial.println(" V");
    
    Serial.println("\n[Cálculos Célula Calibrada (Basados en ADS)]");
    Serial.print("Corriente Isc            : "); Serial.print(fv.Isc, 3); Serial.println(" A");
    Serial.print("Irradiancia (G)          : "); Serial.print(fv.G, 2); Serial.println(" W/m2");
    Serial.print("Temp Célula (Tc_NOCT)    : "); Serial.print(fv.Tc_NOCT, 2); Serial.println(" °C");

    Serial.println("\n[Inversor Huawei SUN2000]");
    Serial.printf("Tensión PV2              : %7.2f V\n", inv.v_pv2);
    Serial.printf("Corriente PV2            : %7.2f A\n", inv.i_pv2);
    Serial.printf("Potencia DC (Entrada)    : %7d W\n", inv.p_dc_in);
    Serial.printf("Potencia AC (Salida)     : %7d W\n", inv.p_ac_out);
    if (inv.p_dc_in > 0) {
      float eff = ((float)inv.p_ac_out / (float)inv.p_dc_in) * 100.0f;
      Serial.printf("Eficiencia Instantánea   : %7.1f %%\n", eff);
    }

    Serial.println("\n[Mercado Eléctrico]");
    Serial.print("Precio PVPC Actual       : "); Serial.print(precioActualKWh, 4); Serial.println(" EUR/kWh");
    Serial.println("---------------------------------------------");
}