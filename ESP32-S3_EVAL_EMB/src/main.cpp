#include <Arduino.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "scaler_params.h"

// Elegir modelo a usar
#define USE_LSTM   // o #define USE_GRU

#ifdef USE_LSTM
  #include "LSTM_model.h"
  #define MODEL_DATA lstm_model_data
  #define MODEL_LEN  lstm_model_data_len
  #define MODEL_NAME "LSTM"
#else
  #include "GRU_model.h"
  #define MODEL_DATA gru_model_data
  #define MODEL_LEN  gru_model_data_len
  #define MODEL_NAME "GRU"
#endif

// ─── TFLite Micro configuración ───────────────────────────────────────
// Tamaño del tensor arena — ajustar según el modelo
// LSTM 32 unidades seq=18: ~80KB
// GRU  32 unidades seq=18: ~60KB
constexpr int TENSOR_ARENA_SIZE = 250 * 1024;  // 250 KB, ajustar si falla
alignas(16) static uint8_t tensor_arena[TENSOR_ARENA_SIZE];

static const tflite::Model* tfl_model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* input_tensor  = nullptr;
static TfLiteTensor* output_tensor = nullptr;

// ─── Buffer circular de la ventana temporal ───────────────────────────
// Almacena los últimos SEQ_LENGTH pasos (cada paso = N_FEATURES valores)
float window_buffer[SEQ_LENGTH][N_FEATURES];
int   window_head   = 0;    // índice del paso más antiguo
bool  window_full   = false;
int   steps_received = 0;

// ─── Variables de medición de latencia ────────────────────────────────
uint32_t t_start_us, t_end_us;

// ─── Inicialización TFLite Micro ──────────────────────────────────────
bool init_tflite() {
    tfl_model = tflite::GetModel(MODEL_DATA);
    if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("[ERROR] Versión de modelo incompatible");
        return false;
    }

    // ─── CAMBIO CRÍTICO PARA EVALUACIÓN ──────────────────────────────────
    // Eliminamos el resolver mutable para no registrar las ops a mano:
    // static tflite::MicroMutableOpResolver<10> resolver;
    // resolver.AddFullyConnected(); ... etc.
    
    // Añadimos el resolvedor universal de todos los operadores:
    static tflite::AllOpsResolver resolver;
    // ─────────────────────────────────────────────────────────────────────

    // ─── Creamos el notificador de errores ───
    static tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    static tflite::MicroInterpreter static_interpreter(
        tfl_model, resolver, tensor_arena, TENSOR_ARENA_SIZE, error_reporter);
    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("[ERROR] AllocateTensors falló — aumenta TENSOR_ARENA_SIZE");
        return false;
    }

    input_tensor  = interpreter->input(0);
    output_tensor = interpreter->output(0);

    Serial.printf("[OK] Modelo %s cargado con AllOpsResolver\n", MODEL_NAME);
    Serial.printf("     Input shape:  [1, %d, %d]\n", SEQ_LENGTH, N_FEATURES);
    return true;
}

// ─── Añadir un paso temporal al buffer circular ────────────────────────
void push_step(float features[N_FEATURES]) {
    for (int i = 0; i < N_FEATURES; i++) {
        window_buffer[window_head][i] = features[i];
    }
    window_head = (window_head + 1) % SEQ_LENGTH;
    steps_received++;
    if (steps_received >= SEQ_LENGTH) window_full = true;
}

// ─── Copiar ventana al tensor de entrada ──────────────────────────────
void fill_input_tensor() {
    float* inp = input_tensor->data.f;
    // El buffer circular empieza en window_head (el más antiguo)
    for (int t = 0; t < SEQ_LENGTH; t++) {
        int idx = (window_head + t) % SEQ_LENGTH;
        for (int f = 0; f < N_FEATURES; f++) {
            inp[t * N_FEATURES + f] = window_buffer[idx][f];
        }
    }
}

// ─── Ejecutar inferencia y devolver predicción en W ───────────────────
float run_inference() {
    fill_input_tensor();

    t_start_us = micros();
    if (interpreter->Invoke() != kTfLiteOk) {
        Serial.println("[ERROR] Invoke falló");
        return -1.0f;
    }
    t_end_us = micros();

    float pred_norm = output_tensor->data.f[0];
    // Clipping físico: la potencia nunca es negativa
    pred_norm = max(pred_norm, 0.0f);
    return denormalize_output(pred_norm);
}

// ─── Setup ────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== Sistema de predicción fotovoltaica ESP32-S3 ===");
    Serial.printf("Modelo: %s | seq_length=%d | features=%d\n",
                  MODEL_NAME, SEQ_LENGTH, N_FEATURES);

    if (!init_tflite()) {
        Serial.println("[FATAL] No se pudo inicializar TFLite. Sistema detenido.");
        while(true) delay(1000);
    }

    Serial.println("\nEsperando datos por Serial (formato CSV):");
    Serial.println("hora_sin,hora_cos,mes_sin,mes_cos,G_Glob,Ta,Hum_Rel,Tc");
}

// ─── Loop ─────────────────────────────────────────────────────────────
void loop() {
    // Protocolo de comunicación con Python vía pyserial:
    // Python envía una línea CSV con 8 valores float
    // ESP32 responde con: prediccion_W,latencia_us
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line == "PING") {
            Serial.println("PONG");
            return; // Cortamos aquí para que no intente parsear esto como CSV
        }

        // Parsear los 8 valores
        float raw_features[N_FEATURES];
        int   parsed = 0;
        char  buf[256];
        line.toCharArray(buf, sizeof(buf));
        char* token = strtok(buf, ",");

        while (token != nullptr && parsed < N_FEATURES) {
            raw_features[parsed++] = atof(token);
            token = strtok(nullptr, ",");
        }

        if (parsed != N_FEATURES) {
            Serial.println("ERROR:formato_incorrecto");
            return;
        }

        // Normalizar features con el MinMaxScaler
        float norm_features[N_FEATURES];
        for (int i = 0; i < N_FEATURES; i++) {
            norm_features[i] = normalize(raw_features[i], i);
            norm_features[i] = constrain(norm_features[i], 0.0f, 1.0f);
        }

        // Añadir al buffer circular
        push_step(norm_features);

        if (!window_full) {
            // Aún no hay suficientes pasos para inferencia
            Serial.printf("BUFFERING:%d/%d\n", steps_received, SEQ_LENGTH);
            return;
        }

        // Ejecutar inferencia
        float prediccion = run_inference();
        uint32_t latencia = t_end_us - t_start_us;

        // Responder a Python
        Serial.printf("PRED:%.2f,LAT:%lu\n", prediccion, latencia);
    }
}