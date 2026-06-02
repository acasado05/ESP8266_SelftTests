#ifndef SCALER_PARAMS_H
#define SCALER_PARAMS_H

// ─── Configuración de la Ventana Temporal ─────────────────────────────
constexpr int SEQ_LENGTH = 18;
constexpr int N_FEATURES = 8;

// ─── Parámetros MinMaxScaler de Entrada (X) ───────────────────────────
// Orden de features: hora_sin, hora_cos, mes_sin, mes_cos, G_Glob, Ta, Hum_Rel, Tc
const float X_MIN[N_FEATURES] = {
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f, 
    0.0f, 
    -3.7f, 
    6.3f, 
    -3.5f
};

const float X_MAX[N_FEATURES] = {
    1.0f, 
    1.0f, 
    1.0f, 
    0.8660254037844387f, 
    1035.4f, 
    38.8f, 
    100.0f, 
    63.3f
};

// ─── Parámetros MinMaxScaler de Salida (y) ────────────────────────────
// Target: Pot_inv (Vatios)
constexpr float Y_MIN = 0.0f;
constexpr float Y_MAX = 4549.6f;

// ─── Funciones de Conversión ──────────────────────────────────────────

// Normaliza un valor de entrada crudo a la escala dictada por el MinMaxScaler
inline float normalize(float val, int feature_idx) {
    if (feature_idx < 0 || feature_idx >= N_FEATURES) return val;
    
    float range = X_MAX[feature_idx] - X_MIN[feature_idx];
    if (range == 0.0f) return 0.0f; // Protección contra división por cero
    
    return (val - X_MIN[feature_idx]) / range;
}

// Desescala la predicción de la red neuronal al valor físico real (Vatios)
inline float denormalize_output(float val) {
    return val * (Y_MAX - Y_MIN) + Y_MIN;
}

#endif // SCALER_PARAMS_H