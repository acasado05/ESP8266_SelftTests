#include <Arduino.h>
#include <ModbusMaster.h>

// --- Configuración Basada en el Código Exitoso ---
#define MODBUS_ID          100      
#define MODBUS_BAUD        9600     // Bajamos a 9600 para máxima estabilidad
#define PIN_RX2            16
#define PIN_TX2            17

ModbusMaster modbus;

// Variables de almacenamiento
float powerAC = 0;
float mppt1_V = 0, mppt1_I = 0;
float mppt2_V = 0, mppt2_I = 0;

// --- Funciones de limpieza de ECO (Imprescindibles para tu módulo rojo) ---
void preTransmission() {
    while (Serial2.available()) Serial2.read();
}
void postTransmission() {
    delay(5); 
    while (Serial2.available()) Serial2.read();
}

void setup() {
    Serial.begin(115200);
    // UART2 a 9600 baudios
    Serial2.begin(MODBUS_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    
    modbus.begin(MODBUS_ID, Serial2);
    modbus.preTransmission(preTransmission);
    modbus.postTransmission(postTransmission);

    Serial.println("--- Modo Secuencial Fronius (9600 bps) ---");
}

void loop() {
    Serial.println("\n--- Iniciando ronda de lectura ---");

    // 1. LECTURA POTENCIA AC (Registro Legacy 499)
    // Leemos 2 registros para obtener un valor de 32 bits
    uint8_t res = modbus.readHoldingRegisters(498, 2); // 499 en base-0 es 498
    if (res == modbus.ku8MBSuccess) {
        uint32_t rawPower = ((uint32_t)modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1);
        powerAC = rawPower * 0.001; // Factor de escala kW según código compañero
        Serial.printf("[AC] Potencia: %.3f kW\n", powerAC);
    } else {
        Serial.printf("[AC] Error 0x%02X\n", res);
    }

    delay(500); // Pausa de "respiro" para el inversor

    // 2. LECTURA MPPT 1 (Registro SunSpec 40282)
    res = modbus.readHoldingRegisters(40282, 2); 
    if (res == modbus.ku8MBSuccess) {
        mppt1_I = modbus.getResponseBuffer(0) * 0.01;
        mppt1_V = modbus.getResponseBuffer(1) * 0.01;
        Serial.printf("[MPPT1] V: %.2fV, I: %.2fA\n", mppt1_V, mppt1_I);
    } else {
        Serial.printf("[MPPT1] Error 0x%02X\n", res);
    }

    delay(500); // Otra pausa

    // 3. LECTURA MPPT 2 (Registro SunSpec 40302)
    res = modbus.readHoldingRegisters(40302, 2);
    if (res == modbus.ku8MBSuccess) {
        mppt2_I = modbus.getResponseBuffer(0) * 0.01;
        mppt2_V = modbus.getResponseBuffer(1) * 0.01;
        Serial.printf("[MPPT2] V: %.2fV, I: %.2fA\n", mppt2_V, mppt2_I);
    } else {
        Serial.printf("[MPPT2] Error 0x%02X\n", res);
    }

    Serial.println("--- Ronda finalizada, esperando 10 segundos ---");
    delay(10000); // Esperamos 10 segundos como tu compañero
}