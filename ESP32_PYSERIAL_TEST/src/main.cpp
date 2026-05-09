#include <Arduino.h>

// --- Estructura de Telemetría (Protocolo Propietario) ---
// Usamos __attribute__((packed)) para evitar que el compilador añada bytes de relleno
struct __attribute__((packed)) TramaDatos {
    float v_pv2;      // f
    float i_pv2;      // f
    int32_t p_dc_in;  // i
    int32_t p_ac_out; // i
    float e_hoy;      // f
    float v_shunt;    // f
    float v_temp_c;   // f
    float t_amb;      // f
    float h_amb;      // f
};

// --- Variables y Configuración ---
TramaDatos telemetria;
uint32_t lastEnvioPC = 0;
const uint32_t INTERVALO_ENVIO = 2000; // Enviar cada 2 segundos

// Función para enviar la trama con envoltorio binario
void enviarTramaPC(uint8_t tipo, uint8_t* payload, uint8_t len) {
    uint8_t checksum = 0;
    
    Serial.write(0xAA);      // Inicio de trama
    Serial.write(tipo);      // Identificador de tipo de mensaje
    Serial.write(len);       // Cuántos bytes de datos vienen
    
    for(uint8_t i = 0; i < len; i++) {
        Serial.write(payload[i]);
        checksum += payload[i]; // Checksum simple: suma de bytes
    }
    
    Serial.write(checksum);  // Byte de validación
    Serial.write(0x55);      // Fin de trama
}

void setup() {
    // USB para comunicación con Python
    Serial.begin(115200);
    
}

void loop() {
    // Simulamos la captura de datos (aquí usarías tus funciones de lectura reales)
    if (millis() - lastEnvioPC > INTERVALO_ENVIO) {
        lastEnvioPC = millis();

        // Rellenamos la estructura con los datos que ya sabes obtener
        telemetria.v_pv2 = 385.2; 
        telemetria.i_pv2 = 4.5;
        telemetria.p_dc_in = 1750;
        telemetria.p_ac_out = 1710;
        telemetria.e_hoy = 12.45;
        telemetria.v_shunt = 0.045672;   // Valor del ADS1115
        telemetria.v_temp_c = 1.254;    
        telemetria.t_amb = 24.5;
        telemetria.h_amb = 45.2;

        // Enviamos la estructura completa al PC
        enviarTramaPC(0x01, (uint8_t*)&telemetria, sizeof(telemetria));
    }
}