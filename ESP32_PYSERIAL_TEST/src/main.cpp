#include <Arduino.h>

// --- CONFIGURACIÓN DEL BUFFER CIRCULAR (TFG) ---
const int FILAS_HISTORICO = 18;  // 3 horas / 10 min
const int INPUTS_MODELO = 8;     // Número de características

float bufferCircular[FILAS_HISTORICO][INPUTS_MODELO];
int indiceEscritura = 0;         // Puntero del buffer circular
int elementosEnBuffer = 0;       // Contador de control

// --- Estructura de Telemetría (ESP32 -> PC) ---
struct __attribute__((packed)) TramaDatos {
    float v_pv2;      
    float i_pv2;      
    int32_t p_dc_in;  
    int32_t p_ac_out; 
    float e_hoy;      
    float v_shunt;    
    float v_temp_c;   
    float t_amb;      
    float h_amb;      
};

TramaDatos telemetria;
uint32_t lastEnvioPC = 0;
const uint32_t INTERVALO_ENVIO = 2000; 

// Función genérica para enviar paquetes binarios encapsulados
void enviarTramaPC(uint8_t tipo, uint8_t* payload, uint8_t len) {
    uint8_t checksum = 0;
    Serial.write(0xAA);      
    Serial.write(tipo);      
    Serial.write(len);       
    for(uint8_t i = 0; i < len; i++) {
        Serial.write(payload[i]);
        checksum += payload[i]; 
    }
    Serial.write(checksum);  
    Serial.write(0x55);      
}

// Función para procesar e insertar los 8 inputs recibidos del PC
void procesarInputsRecibidos(uint8_t* payload) {
    // Reinterpretamos el buffer de bytes como un array de 8 floats
    float* inputs = (float*)payload;

    // Insertamos los datos en la posición actual del buffer circular
    for (int i = 0; i < INPUTS_MODELO; i++) {
        bufferCircular[indiceEscritura][i] = inputs[i];
    }

    // --- NUEVO: Capturamos la fila que acabamos de escribir antes de mover el puntero ---
    uint8_t filaModificada = indiceEscritura;

    // Avanzamos el puntero de forma circular
    indiceEscritura = (indiceEscritura + 1) % FILAS_HISTORICO;
    
    if (elementosEnBuffer < FILAS_HISTORICO) {
        elementosEnBuffer++;
    }

    // --- NUEVO: Enviar el volcado de la fila hacia Python (Tipo 0x03) ---
    // Tamaño: 1 byte (índice) + 32 bytes (8 floats x 4 bytes) = 33 bytes
    uint8_t bufferDebug[33]; 
    
    bufferDebug[0] = filaModificada; // El primer byte guarda la posición (0 a 17)
    
    // Copiamos los 32 bytes de la fila del buffer circular a la posición de memoria trasera
    memcpy(&bufferDebug[1], &bufferCircular[filaModificada][0], 32);
    
    // Despachamos la trama binaria con el identificador de tipo 0x03
    enviarTramaPC(0x03, bufferDebug, 33);


    // Feedback por monitor serie (Mantenemos tus prints originales)
    Serial.printf("\n[BUFFER] Guardada fila en indice %d. Total en buffer: %d/18\n", 
                  filaModificada, 
                  elementosEnBuffer);
    Serial.printf("[BUFFER] Primeros 2 inputs guardados: %.2f, %.2f\n", inputs[0], inputs[1]);
}

void setup() {
    // Configuración para el USB Nativo del S3 oficial
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 4000)) {
        delay(10);
    }
}

void loop() {
    // 1. RECEPCIÓN Y DESEMPAQUETADO (PC -> ESP32)
    if (Serial.available() > 0) {
        if (Serial.read() == 0xAA) { // Byte de inicio
            uint8_t tipo = Serial.read();
            uint8_t len = Serial.read();
            
            // Esperamos a que el payload completo, el checksum y el fin estén en la caché de la UART
            if (tipo == 0x02 && len == (INPUTS_MODELO * sizeof(float))) {
                while (Serial.available() < (len + 2)) {
                    delayMicroseconds(50); // Pequeña espera de cortesía de hardware
                }

                uint8_t payload[32]; // 8 * 4 bytes = 32
                uint8_t checksumCalculado = 0;

                for (uint8_t i = 0; i < len; i++) {
                    payload[i] = Serial.read();
                    checksumCalculado += payload[i];
                }

                uint8_t checksumRecibido = Serial.read();
                uint8_t fin = Serial.read();

                // Validación estricta de la trama del PC
                if (checksumCalculado == checksumRecibido && fin == 0x55) {
                    procesarInputsRecibidos(payload);
                    
                    // Respondemos INMEDIATAMENTE enviando la telemetría actual como "eco" de confirmación
                    enviarTramaPC(0x01, (uint8_t*)&telemetria, sizeof(telemetria));
                }
            }
        }
    }

    // 2. ENVÍO PERIÓDICO ASÍNCRONO
    if (millis() - lastEnvioPC > INTERVALO_ENVIO) {
        lastEnvioPC = millis();

        // Datos simulados de telemetría ordinaria
        telemetria.v_pv2 = 385.2; 
        telemetria.i_pv2 = 4.5;
        telemetria.p_dc_in = 1750;
        telemetria.p_ac_out = 1710;
        telemetria.e_hoy = 12.45;
        telemetria.v_shunt = 0.045672;   
        telemetria.v_temp_c = 1.254;    
        telemetria.t_amb = 24.5;
        telemetria.h_amb = 45.2;

        enviarTramaPC(0x01, (uint8_t*)&telemetria, sizeof(telemetria));
    }
}