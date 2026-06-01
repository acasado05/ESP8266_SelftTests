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
#include <SPI.h>
#include <SD.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include <scaler_params.h>
#include "LSTM_model.h"

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
#define REG_E_DAILY       32214   // U32, Gain 100 (kWh) - Energía Diaria
#define REG_E_TOTAL       32216   // U32, Gain 100 (kWh) - Energía Total

// --- Configuración MicroSD ---
#define SD_SCK  12
#define SD_MISO 13
#define SD_MOSI 11
#define SD_CS   10

/* Configurar pines I2C */
const int sdaPin = 8;
const int sclPin = 9;

// ─── BUFFER CIRCULAR ───
float circular_buffer[SEQ_LENGTH][N_FEATURES];
String buffer_timestamps[SEQ_LENGTH];
int buffer_count = 0;
bool buffer_lleno = false;

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

// --- Temporizadores no bloqueantes ---
unsigned long lastSerialTime = 0;
unsigned long lastSDTime = 0;
const unsigned long serialInterval = 10000; // 10 segundos
const unsigned long sdInterval = 60000;     // 60 segundos (1 minuto)

// Sensores
Adafruit_ADS1115 ads;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Variables microSD
SPIClass spiSD(FSPI);
const char* logFile = "/datalogger_tfg.csv";

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
  float p_dc_in;
  float p_ac_out;
  float e_daily;
  float e_total;
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

// ─── VARIABLES GLOBALES TFLITE ────────────────────────────────────────
// 50KB de RAM reservados estrictamente para las operaciones matriciales
constexpr int TENSOR_ARENA_SIZE = 60 * 1024; 
alignas(16) static uint8_t tensor_arena[TENSOR_ARENA_SIZE];

static const tflite::Model* tfl_model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;

// Estas variables DEBEN ser globales para que tu función 
// ejecutar_inferencia_y_calibrar() pueda acceder a ellas
TfLiteTensor* input_tensor = nullptr;
TfLiteTensor* output_tensor = nullptr;

// --- Declaración de Funciones ---
bool init_tflite() ;
void procesar_y_normalizar(int mes, int hora, float g_glob, float ta, float hum_rel, float tc, float* vector_salida);
void actualizar_buffer_circular(float nuevo_vector[N_FEATURES], String timestamp);
float ejecutar_inferencia_y_calibrar();
void debug_imprimir_buffer_completo();
void wifiSetUp();
void setNTP();
String getTimeStamp();
void obtenerPrecioESIOS();
MedidasAmbientales realizarMedida(void);
DatosFotovoltaicos calcularParametrosSolares(float ambTemp);
DatosInversor leerInversorHuawei();
void SDsetup();
void saveDataSD(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv, const DatosInversor& inv);
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

  // Configuración de MicroSD
  SDsetup();

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

  Serial.println("\n--- Arrancando Motor de Inteligencia Artificial ---");
    if (!init_tflite()) {
        Serial.println("[FATAL] El sistema embebido se ha detenido por fallo en la IA.");
        while(true) delay(1000); // Bucle infinito de seguridad si la IA falla
    }
    Serial.println("--- Sistema predictivo listo y operativo ---\n");

}

void loop() {
  unsigned long currentMillis = millis();

  // 1. ESIOS (se actualiza de fondo cada 10 minutos)
  if (currentMillis - lastTimeRequest >= timerDelay) {
      lastTimeRequest = currentMillis;
      obtenerPrecioESIOS();
  }

  // 2. Bloque principal: Dispara cada 10 segundos
  if (currentMillis - lastSerialTime >= serialInterval) {
    lastSerialTime = currentMillis;

    // Obtener medidas físicas en este instante preciso
    MedidasAmbientales misMedidasAmb = realizarMedida();
    DatosFotovoltaicos misDatosFV = calcularParametrosSolares(misMedidasAmb.tempAmbFinal);
    DatosInversor misDatosInv = leerInversorHuawei();

    // Imprimir por Monitor Serie
    logDatosSerial(misMedidasAmb, misDatosFV, misDatosInv);

      // Extraemos el mes (1-12) y la hora (0-23) del struct timeinfo (NTP)
    // Nota: tm_mon va de 0 (Enero) a 11 (Diciembre), por lo que sumamos 1.
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("[IA WARNING] Fallo NTP. Usando hora por defecto.");
        timeinfo.tm_mon = 4;
        timeinfo.tm_hour = 12;
    }
    int mes_actual = timeinfo.tm_mon + 1; 
    int hora_actual = timeinfo.tm_hour;

    // 2. PROTECCIONES DE SENSORES
    float g_seguro   = isnan(misDatosFV.G) ? 0.0f : misDatosFV.G;
    float ta_seguro  = isnan(misMedidasAmb.tempAHT) ? 25.0f : misMedidasAmb.tempAHT;
    float hum_seguro = isnan(misMedidasAmb.humAHT) ? 50.0f : misMedidasAmb.humAHT;
    float tc_seguro  = isnan(misDatosFV.Tc_NOCT) ? ta_seguro : misDatosFV.Tc_NOCT; 

    // 3. PIPELINE IA
    float vector_ia_actual[N_FEATURES];
    procesar_y_normalizar(mes_actual, hora_actual, g_seguro, ta_seguro, hum_seguro, tc_seguro, vector_ia_actual);
    
    // 4. ¡EL TOQUE MAESTRO! Obtenemos la etiqueta usando tu función
    String etiqueta_actual = getTimeStamp(); 
    
    // Y se la pasamos al buffer para que la guarde junto con los datos
    actualizar_buffer_circular(vector_ia_actual, etiqueta_actual);

    // 5. Imprimir Matriz Completa
    debug_imprimir_buffer_completo();

    if (buffer_lleno) {
        // Ejecutamos el modelo
        float prediccion_W = ejecutar_inferencia_y_calibrar();
        
        // Aquí llamas a tu función de guardar en la MicroSD
        // Ej: guardar_en_SD(etiqueta_actual, g_seguro, ta_seguro, tc_seguro, prediccion_W);
        
    } else {
        Serial.println("[INFO] Búfer llenándose. No se realiza inferencia todavía.");
        // Si quieres, aquí puedes guardar en la SD con predicción = 0.0
    }

    // 3. Sub-bloque SD: Dispara cada 60 segundos (1 minuto)
    if (currentMillis - lastSDTime >= sdInterval) {
        lastSDTime = currentMillis;
        
        // Guardar en la tarjeta SD los mismos datos recién impresos
        saveDataSD(misMedidasAmb, misDatosFV, misDatosInv);
    }
  }
}

// --------------------------------------------------------
// IMPLEMENTACIÓN DE FUNCIONES
// --------------------------------------------------------

// ─── CONFIGURACIÓN DEL MOTOR DE INFERENCIA ────────────────────────────
bool init_tflite() {
    // 1. Cargar el array de bytes generado desde Python
    // Asegúrate de que "lstm_model_data" coincide con el nombre del array en tu LSTM_model.h
    tfl_model = tflite::GetModel(lstm_model_data); 
    
    if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("[ERROR CRÍTICO] Versión del modelo TFLite incompatible.");
        return false;
    }

    // 2. Instanciar el resolvedor universal de operaciones
    static tflite::AllOpsResolver resolver;
    
    // 3. Configurar reporte de errores
    static tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    // 4. Crear el intérprete
    static tflite::MicroInterpreter static_interpreter(
        tfl_model, resolver, tensor_arena, TENSOR_ARENA_SIZE, error_reporter);
    interpreter = &static_interpreter;

    // 5. Asignar memoria estática a los tensores
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("[ERROR CRÍTICO] AllocateTensors falló. Aumenta TENSOR_ARENA_SIZE.");
        return false;
    }

    // 6. Enlazar los punteros globales de entrada y salida
    input_tensor  = interpreter->input(0);
    output_tensor = interpreter->output(0);

    Serial.printf("[OK] Modelo IA cargado exitosamente.\n");
    Serial.printf("     Memoria Tensor Arena utilizada: %u bytes\n", interpreter->arena_used_bytes());
    
    return true;
}

void procesar_y_normalizar(int mes, int hora, float g_glob, float ta, float hum_rel, float tc, float* vector_salida) {
    // 1. Transformación Trigonométrica (en radianes)
    float hora_sin = sin(hora * (2.0f * PI / 24.0f));
    float hora_cos = cos(hora * (2.0f * PI / 24.0f));
    float mes_sin  = sin(mes * (2.0f * PI / 12.0f));
    float mes_cos  = cos(mes * (2.0f * PI / 12.0f));

    // 2. Normalización usando tu scaler_params.h
    vector_salida[0] = normalize(hora_sin, 0);
    vector_salida[1] = normalize(hora_cos, 1);
    vector_salida[2] = normalize(mes_sin,  2);
    vector_salida[3] = normalize(mes_cos,  3);
    vector_salida[4] = normalize(g_glob,   4);
    vector_salida[5] = normalize(ta,       5);
    vector_salida[6] = normalize(hum_rel,  6);
    vector_salida[7] = normalize(tc,       7);
    
    // 3. Clipping estricto de seguridad para TFLite
    for(int i = 0; i < N_FEATURES; i++) {
        vector_salida[i] = constrain(vector_salida[i], 0.0f, 1.0f);
    }
}

void actualizar_buffer_circular(float nuevo_vector[N_FEATURES], String etiqueta_tiempo) {
    
    // 1. Desplazar el historial hacia el "pasado"
    for (int i = 0; i < SEQ_LENGTH - 1; i++) {
        for (int j = 0; j < N_FEATURES; j++) {
            circular_buffer[i][j] = circular_buffer[i + 1][j];
        }
        buffer_timestamps[i] = buffer_timestamps[i + 1]; 
    }
    
    // 2. Insertar el vector actual
    for (int j = 0; j < N_FEATURES; j++) {
        circular_buffer[SEQ_LENGTH - 1][j] = nuevo_vector[j];
    }
    
    // 3. Usamos TU etiqueta de tiempo
    buffer_timestamps[SEQ_LENGTH - 1] = etiqueta_tiempo;
    
    // 4. Gestionar el llenado
    if (!buffer_lleno) {
        buffer_count++;
        if (buffer_count >= SEQ_LENGTH) {
            buffer_lleno = true;
        }
    }
}

float ejecutar_inferencia_y_calibrar() {
    
    // 1. Llenar el Tensor de Entrada (Flattening)
    // TFLite espera un array plano (1D) de 144 posiciones (18 x 8).
    int flat_index = 0;
    for (int i = 0; i < SEQ_LENGTH; i++) {
        for (int j = 0; j < N_FEATURES; j++) {
            // Asumimos que 'input_tensor' es tu variable global de TFLite
            input_tensor->data.f[flat_index] = circular_buffer[i][j];
            flat_index++;
        }
    }

    // 2. Ejecutar la Red Neuronal (LSTM)
    unsigned long t_inicio = micros();
    
    TfLiteStatus invoke_status = interpreter->Invoke(); // ¡La magia ocurre aquí!
    
    if (invoke_status != kTfLiteOk) {
        Serial.println("[ERROR CRÍTICO] Fallo al invocar el intérprete TFLite");
        return -1.0f; 
    }
    unsigned long t_fin = micros();
    unsigned long latencia = t_fin - t_inicio;

    // 3. Extraer la predicción normalizada [0, 1]
    float pred_norm = output_tensor->data.f[0];
    pred_norm = max(pred_norm, 0.0f); // Evitar minúsculos rebotes negativos por la noche

    // 4. Desescalar a Vatios (usando tu scaler_params.h)
    float pred_W_bruta = denormalize_output(pred_norm);

    // 5. Aplicar la Calibración Física (Factor de Reducción K)
    // Pon el valor exacto que sacaste con tu script de Python
    const float FACTOR_K = 0.2854f; 
    float pred_W_final = pred_W_bruta * FACTOR_K;

    // Imprimir resultados
    Serial.println("\n[IA INFERENCIA] ==========================================");
    Serial.printf(" Latencia de Inferencia : %lu us (%.2f ms)\n", latencia, latencia/1000.0);
    Serial.printf(" Salida Red (Norm)      : %.4f\n", pred_norm);
    Serial.printf(" Potencia Predicha      : %.1f W\n", pred_W_final);
    Serial.println("==========================================================\n");

    return pred_W_final;
}

void debug_imprimir_buffer_completo() {
    Serial.println("\n=====================================================================");
    Serial.printf(" ESTADO DEL BUFFER CIRCULAR (%d/%d muestras) %s\n", 
                  buffer_count, SEQ_LENGTH, buffer_lleno ? "[LISTO PARA IA]" : "[LLENANDO]");
    Serial.println("=====================================================================");
    
    for (int i = 0; i < SEQ_LENGTH; i++) {
        // Si la fila está vacía, ponemos --:--
        String ts = buffer_timestamps[i];
        if (ts.length() == 0) ts = "--:--";

        // Formato alineado de las etiquetas
        if (i == 0) {
            Serial.printf(" [t-17] (%s) (Antiguo) -> [", ts.c_str());
        } else if (i == SEQ_LENGTH - 1) {
            Serial.printf(" [ t  ] (%s) (Actual)  -> [", ts.c_str());
        } else {
            Serial.printf(" [t-%02d] (%s)           -> [", SEQ_LENGTH - 1 - i, ts.c_str());
        }

        // Imprimir el vector normalizado
        for (int j = 0; j < N_FEATURES; j++) {
            Serial.printf("%5.3f", circular_buffer[i][j]); 
            if (j < N_FEATURES - 1) Serial.print(", ");
        }
        Serial.println("]");
    }
    Serial.println("=====================================================================\n");
}

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
    int32_t raw_dc = (int32_t)res[3] << 24 | (int32_t)res[4] << 16 | (int32_t)res[5] << 8 | (int32_t)res[6];
    inv.p_dc_in = (float)raw_dc;
  }
  delay(100);

  // 3. Leer Potencia Salida AC (I32)
  sendModbusRequest(HUAWEI_ID, 0x03, REG_P_ACTIVE_AC, 2);
  if (readModbusResponse(res, sizeof(res)) >= 9) {
    int32_t raw_ac = (int32_t)res[3] << 24 | (int32_t)res[4] << 16 | (int32_t)res[5] << 8 | (int32_t)res[6];
    inv.p_ac_out = (float)raw_ac;
  }

  // 4. Leer Energía Diaria y Total (4 registros seguidos desde 32214)
  sendModbusRequest(HUAWEI_ID, 0x03, REG_E_DAILY, 4);
  if (readModbusResponse(res, sizeof(res)) >= 13) {
    uint32_t raw_daily = (uint32_t)res[3] << 24 | (uint32_t)res[4] << 16 | (uint32_t)res[5] << 8 | (uint32_t)res[6];
    uint32_t raw_total = (uint32_t)res[7] << 24 | (uint32_t)res[8] << 16 | (uint32_t)res[9] << 8 | (uint32_t)res[10];
    
    inv.e_daily = (float)raw_daily / 100.0f; // La ganancia en estos registros es de 100
    inv.e_total = (float)raw_total / 100.0f;
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

// =============================================================================
// FUNCIONES DE REGISTRO EN MICRO SD
// =============================================================================

void SDsetup() {
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  if (!SD.begin(SD_CS, spiSD, 4000000)) {
    Serial.println("[ERROR]: Problema al montar la tarjeta SD. Verifica cableado y formato FAT32.");
    return;
  }
  
  if (!SD.exists(logFile)) {
    File file = SD.open(logFile, FILE_WRITE);
    if (file) {
      // Cabeceras exactas solicitadas
      file.println("Timestamp,Irradiancia_G,Tc,T_Amb,Hum_Rel,V_PV2,I_PV2,P_DC_W,P_AC_W,E_Daily_kWh,E_Total_kWh,Precio_EUR_kWh");
      file.close();
      Serial.println("[SD] Archivo CSV creado con cabeceras.");
    } else {
      Serial.println("[ERROR] No se pudo crear el archivo CSV.");
    }
  } else {
    Serial.println("[SD] Tarjeta montada. Archivo de registro existente.");
  }
}

void saveDataSD(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv, const DatosInversor& inv) {
  File file = SD.open(logFile, FILE_APPEND);
  if (file) {
    file.print(getTimeStamp()); file.print(",");
    file.print(fv.G, 2); file.print(",");
    file.print(fv.Tc_NOCT, 2); file.print(",");
    file.print(amb.tempAmbFinal, 2); file.print(",");
    file.print(amb.humAHT, 2); file.print(",");
    file.print(inv.v_pv2, 2); file.print(",");
    file.print(inv.i_pv2, 2); file.print(",");
    file.print(inv.p_dc_in, 2); file.print(",");
    file.print(inv.p_ac_out, 2); file.print(",");
    file.print(inv.e_daily, 2); file.print(",");
    file.print(inv.e_total, 2); file.print(",");
    file.println(precioActualKWh, 4); // 4 decimales de precisión para los euros + salto línea
    file.close();
    Serial.println(">>> Registro guardado en SD (11 variables incl. Precio).");
  } else {
    Serial.println("[ERROR] Error abriendo el archivo SD para guardar.");
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
    Serial.printf("Potencia DC (Entrada)    : %7.2f W\n", inv.p_dc_in);
    Serial.printf("Potencia AC (Salida)     : %7.2f W\n", inv.p_ac_out);
    if (inv.p_dc_in > 0) {
      float eff = ((float)inv.p_ac_out / (float)inv.p_dc_in) * 100.0f;
      Serial.printf("Eficiencia Instantánea   : %7.1f %%\n", eff);
    }

    Serial.println("\n[Mercado Eléctrico]");
    Serial.print("Precio PVPC Actual       : "); Serial.print(precioActualKWh, 4); Serial.println(" EUR/kWh");
    Serial.println("---------------------------------------------");
}