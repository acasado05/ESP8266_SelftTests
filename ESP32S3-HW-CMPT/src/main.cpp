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
int window_head = 0;

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
const unsigned long serialInterval = 600000; // 10 segundos

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
void saveDataSD(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv, const DatosInversor& inv, float prediccion_ia);
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

  // 2. Bloque principal unificado: Adquisición, IA y Guardado SD
  // IMPORTANTE: Recuerda cambiar arriba en tus variables globales:
  // const unsigned long serialInterval = 600000; // Para que dispare cada 10 minutos
  if (currentMillis - lastSerialTime >= serialInterval) {
    lastSerialTime = currentMillis;

    // Obtener medidas físicas en este instante preciso
    MedidasAmbientales misMedidasAmb = realizarMedida();
    DatosFotovoltaicos misDatosFV = calcularParametrosSolares(misMedidasAmb.tempAmbFinal);
    DatosInversor misDatosInv = leerInversorHuawei();

    // Imprimir por Monitor Serie
    logDatosSerial(misMedidasAmb, misDatosFV, misDatosInv);

    // Extraemos el mes (1-12) y la hora (0-23) del struct timeinfo (NTP)
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

    // 6. INFERENCIA Y GUARDADO SINCRONIZADO EN SD
    // Inicializamos a 0 por si el buffer aún no está lleno
    float prediccion_W = 0.0f; 

    if (buffer_lleno) {
        // Ejecutamos el modelo y guardamos el resultado en la variable
        prediccion_W = ejecutar_inferencia_y_calibrar();
    } else {
        Serial.println("[INFO] Búfer llenándose. No se realiza inferencia todavía.");
    }

    // Guardar ABSOLUTAMENTE TODO en la tarjeta SD de una sola vez
    // (Si el buffer se está llenando, guardará 0.0 W en la columna de IA)
    saveDataSD(misMedidasAmb, misDatosFV, misDatosInv, prediccion_W);

  } // Fin del bloque principal
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
    if (!buffer_lleno) {
        // Fase de cebado (Llenando las primeras 18 posiciones)
        for (int f = 0; f < N_FEATURES; f++) {
            circular_buffer[buffer_count][f] = nuevo_vector[f];
        }
        buffer_timestamps[buffer_count] = etiqueta_tiempo;
        buffer_count++;
        
        if (buffer_count >= SEQ_LENGTH) {
            buffer_lleno = true;
            window_head = 0; // El índice 0 pasa a ser el dato más antiguo
        }
    } else {
        // Modo permanente: Sobrescribir solo la posición más antigua
        for (int f = 0; f < N_FEATURES; f++) {
            circular_buffer[window_head][f] = nuevo_vector[f];
        }
        buffer_timestamps[window_head] = etiqueta_tiempo;
        
        // Avanzar el puntero circularmente
        window_head = (window_head + 1) % SEQ_LENGTH;
    }
}

float ejecutar_inferencia_y_calibrar() {
    // 1. Llenar el Tensor de Entrada usando el Ring Buffer
    float* inp = input_tensor->data.f;
    for (int t = 0; t < SEQ_LENGTH; t++) {
        // Desenvolvemos la matriz empezando desde el puntero más antiguo
        int idx = (window_head + t) % SEQ_LENGTH;
        for (int f = 0; f < N_FEATURES; f++) {
            inp[t * N_FEATURES + f] = circular_buffer[idx][f];
        }
    }

    // 2. Ejecutar la Red Neuronal (LSTM)
    unsigned long t_inicio = micros();
    if (interpreter->Invoke() != kTfLiteOk) {
        Serial.println("[ERROR CRÍTICO] Fallo al invocar el intérprete TFLite");
        return -1.0f; 
    }
    unsigned long t_fin = micros();
    unsigned long latencia = t_fin - t_inicio;

    // 3. Extraer y desescalar
    float pred_norm = output_tensor->data.f[0];
    pred_norm = max(pred_norm, 0.0f); // Evitar potencias negativas
    float pred_W_bruta = denormalize_output(pred_norm);

    // 4. Calibración K
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
    
    // Iteramos 't' como tiempo cronológico, no como índice de memoria
    for (int t = 0; t < SEQ_LENGTH; t++) {
        
        // Evitamos imprimir filas vacías si el buffer aún se está cebando
        if (!buffer_lleno && t >= buffer_count) break;

        // 1. CÁLCULO DEL ÍNDICE FÍSICO
        // Si está lleno, leemos desde window_head. Si no, leemos normal.
        int idx = buffer_lleno ? ((window_head + t) % SEQ_LENGTH) : t;
        
        String ts = buffer_timestamps[idx];
        if (ts.length() == 0) ts = "--:--";

        // 2. ETIQUETAS TEMPORALES CORREGIDAS
        // Calculamos los saltos hacia atrás desde la muestra más reciente
        int offset = buffer_lleno ? (SEQ_LENGTH - 1 - t) : (buffer_count - 1 - t);

        if (offset == 0) {
            // Es la muestra más reciente (offset = 0)
            Serial.printf(" [ t  ] (%s) (Actual)  -> [", ts.c_str());
        } else if (t == 0) {
            // Es la muestra más antigua registrada hasta ahora
            Serial.printf(" [t-%02d] (%s) (Antiguo) -> [", offset, ts.c_str());
        } else {
            // Muestras intermedias
            Serial.printf(" [t-%02d] (%s)           -> [", offset, ts.c_str());
        }

        // 3. IMPRESIÓN DE LA MATRIZ
        for (int j = 0; j < N_FEATURES; j++) {
            Serial.printf("%5.3f", circular_buffer[idx][j]); 
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
      file.println("Timestamp,Irradiancia_G,Tc,T_Amb,Hum_Rel,V_PV2,I_PV2,P_DC_W,P_AC_W,E_Daily_kWh,E_Total_kWh,Precio_EUR_kWh,Prediccion_IA_W");
      file.close();
      Serial.println("[SD] Archivo CSV creado con cabeceras.");
    } else {
      Serial.println("[ERROR] No se pudo crear el archivo CSV.");
    }
  } else {
    Serial.println("[SD] Tarjeta montada. Archivo de registro existente.");
  }
}

void saveDataSD(const MedidasAmbientales& amb, const DatosFotovoltaicos& fv, const DatosInversor& inv, float prediccion_ia) {
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
    file.print(precioActualKWh, 4); file.print(",");
    file.println(prediccion_ia, 2); // 2 decimales de precisión para la predicción
    file.close();
    Serial.println(">>> Registro guardado en SD (12 variables incl. IA).");
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