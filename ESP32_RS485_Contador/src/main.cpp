#include <Arduino.h>

// ============================================================================
//  TEST_ORBIS_PLATFORMIO.cpp
//  Adaptado por: Aitor (Basado en código de Pablo Gilsanz Herrero)
// ============================================================================

// ── Pines de hardware ────────────────────────────────────────────────────────
#define RS485_TX    17    // GPIO17 -> Módulo DI
#define RS485_RX    16    // GPIO16 <- Módulo RO
#define RE_DE_PIN   4     // GPIO4  -> Control RE/DE (Unidos) [NUEVO]

// ── Configuración Modbus ──────────────────────────────────────────────────────
#define ORBIS_ID     190    // ID del contador [cite: 5]
#define BAUDRATE    9600    // [cite: 5]
#define TIMEOUT_MS  1000    // [cite: 5]

#define DEBUG_MODE  true
#define READ_INTERVAL_MS  3000

// ── Prototipos de funciones ───────────────────────────────────────────────────
uint16_t crc16(const uint8_t *data, uint8_t len);
void printHex(const uint8_t *buf, uint8_t len);
void sendModbusRequest(uint8_t slaveId, uint8_t funcCode, uint16_t regAddr, uint16_t numRegs);
uint8_t readModbusResponse(uint8_t *buf, uint8_t maxLen);
bool validateResponse(const uint8_t *buf, uint8_t len, uint8_t expectedId, uint8_t expectedFc);
float readU16(uint16_t regAddr, float gain);
float readU32reg(uint16_t regAddr, float gain);
void readAllOrbis();
void printSummary();

// ── Variables globales ────────────────────────────────────────────────────────
float ct_voltage_v  = 0.0f;
float ct_current_a  = 0.0f;
float ct_power_kw   = 0.0f;
float ct_energy_kwh = 0.0f;
uint32_t lastReadTime = 0;

// =============================================================================
//  CRC-16 Modbus [cite: 8]
// =============================================================================
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

// =============================================================================
//  sendModbusRequest (CON GESTIÓN DE GPIO4)
// =============================================================================
void sendModbusRequest(uint8_t slaveId, uint8_t funcCode, uint16_t regAddr, uint16_t numRegs) {
  while (Serial2.available()) Serial2.read(); 

  uint8_t frame[8];
  frame[0] = slaveId;
  frame[1] = funcCode;
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = (numRegs >> 8) & 0xFF;
  frame[5] = numRegs & 0xFF;
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;          
  frame[7] = (crc >> 8) & 0xFF;

  digitalWrite(RE_DE_PIN, HIGH); // Activar Transmisión
  Serial2.write(frame, 8);
  Serial2.flush();               // Esperar a que salga el último bit 
  
  // Pequeño retardo de seguridad antes de bajar el pin
  delayMicroseconds(50); 
  digitalWrite(RE_DE_PIN, LOW);  // Activar Recepción
}

// =============================================================================
//  readModbusResponse
// =============================================================================
uint8_t readModbusResponse(uint8_t *buf, uint8_t maxLen) {
  uint32_t t0 = millis();
  uint8_t idx = 0;
  
  // Esperar respuesta con timeout
  while (!Serial2.available()) {
    if (millis() - t0 > TIMEOUT_MS) return 0;
  }

  t0 = millis();
  while (millis() - t0 < 100) { // Ventana de tiempo para recibir la trama completa
    if (Serial2.available()) {
      byte c = Serial2.read();
      
      // FILTRO CRÍTICO: Si el primer byte es 00, lo ignoramos para evitar el eco/ruido
      if (idx == 0 && c == 0x00) {
        continue; 
      }

      if (idx < maxLen) {
        buf[idx++] = c;
      }
      t0 = millis();
    }
  }

  if (DEBUG_MODE && idx > 0) {
    Serial.print(F("  [RX-Limpia] "));
    printHex(buf, idx);
    Serial.printf(" (%d bytes)\n", idx);
  }
  return idx;
}

// =============================================================================
//  Funciones de lectura de registros [cite: 6]
// =============================================================================
float readU16(uint16_t regAddr, float gain) {
  uint8_t buf[16];
  sendModbusRequest(ORBIS_ID, 0x04, regAddr, 1);
  uint8_t len = readModbusResponse(buf, sizeof(buf));
  if (!validateResponse(buf, len, ORBIS_ID, 0x04)) return NAN;
  uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
  return (float)raw / gain;
}

float readU32reg(uint16_t regAddr, float gain) {
  uint8_t buf[16];
  sendModbusRequest(ORBIS_ID, 0x04, regAddr, 2);
  uint8_t len = readModbusResponse(buf, sizeof(buf));
  if (!validateResponse(buf, len, ORBIS_ID, 0x04)) return NAN;
  uint32_t raw = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8) | (uint32_t)buf[6];
  return (float)raw / gain;
}

// =============================================================================
//  Validación y Resumen
// =============================================================================
bool validateResponse(const uint8_t *buf, uint8_t len, uint8_t expectedId, uint8_t expectedFc) {
  if (len < 5) return false;
  uint16_t crcCalc = crc16(buf, len - 2);
  uint16_t crcRecv = (uint16_t)buf[len-2] | ((uint16_t)buf[len-1] << 8);
  if (crcCalc != crcRecv) return false;
  if (buf[0] != expectedId || (buf[1] & 0x80) || buf[1] != expectedFc) return false;
  return true;
}

void printHex(const uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX); Serial.print(' ');
  }
}

void readAllOrbis() {
  Serial.println(F("\n--- Leyendo ORBIS CONTAX D-6041-BUS ---"));
  
  // Voltaje
  float v = readU16(0x0046, 10.0f);
  if (!isnan(v)) ct_voltage_v = v;
  delay(200); // <--- Aumenta este delay a 200ms entre registros

  // Corriente
  float i = readU16(0x004C, 100.0f);
  if (!isnan(i)) ct_current_a = i;
  delay(200);

  // Potencia
  float p = readU16(0x004F, 100.0f);
  if (!isnan(p)) ct_power_kw = p;
}

void printSummary() {
  Serial.println(F("\n========================================"));
  Serial.printf("  Voltaje   : %6.1f V\n",  ct_voltage_v);
  Serial.printf("  Corriente : %6.2f A\n",  ct_current_a);
  Serial.printf("  Potencia  : %6.3f kW\n", ct_power_kw);
  Serial.printf("  Energia   : %6.3f kWh\n",ct_energy_kwh);
  Serial.println(F("========================================\n"));
}

// =============================================================================
//  SETUP Y LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Configuración del pin de control RE/DE
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Iniciar en modo recepción

  Serial2.begin(BAUDRATE, SERIAL_8N1, RS485_RX, RS485_TX); // [cite: 64]
  Serial.println(F("Sistema iniciado. Gestionando RE/DE en GPIO4."));
  delay(500);
}

void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = millis();
    readAllOrbis();
    printSummary();
  }
}