// ============================================================================
//  main.cpp
//  Autor: Pablo Gilsanz Herrero
//
//  Proyecto PlatformIO — ESP32 DevKit V1 / WROOM-32
//
//  platformio.ini recomendado:
//  ─────────────────────────────────────────────────
//  [env:esp32dev]
//  platform  = espressif32
//  board     = esp32dev
//  framework = arduino
//  monitor_speed = 115200
//  ─────────────────────────────────────────────────
//
//  Hardware:
//    - ESP32 DevKit V1 / WROOM-32
//    - Módulo MAX3485 rojo (DE+/RE cortocircuitados en PCB, sin pin de control)
//    - Contador ORBIS CONTAX D-6041-BUS
//
//  Conexiones:
//  ─────────────────────────────────────────────────────
//   ESP32 3V3          ──→  Módulo VCC
//   ESP32 GND          ──→  Módulo GND
//   ESP32 TX2 (GPIO17) ──→  Módulo TXD
//   ESP32 RX2 (GPIO16) ──→  Módulo RXD
//   Módulo D-/B        ──→  ORBIS terminal B (-)
//   Módulo D+/A        ──→  ORBIS terminal A (+)
//
//  IMPORTANTE — Jumper "20 S":
//    Debe estar QUITADO para que el módulo funcione en half-duplex
//    automático. Con el jumper puesto, DE está fijo a VCC (siempre TX)
//    y el ESP32 nunca podrá recibir la respuesta del contador.
//
//  Sin el jumper, el módulo conmuta TX/RX automáticamente por el
//  silencio en el bus — no se necesita ningún pin de control desde
//  el ESP32.
//
//  Protocolo: Modbus RTU, 9600 bps, 8N1
//  Dirección contador ORBIS: 190 (0xBE) — ajusta si es diferente
// ============================================================================

#include <Arduino.h>    // Obligatorio en PlatformIO (en .ino se incluye implícito)

// ── Pines Serial2 ────────────────────────────────────────────────────────────
#define RS485_TX  17    // GPIO17 → Módulo TXD
#define RS485_RX  16    // GPIO16 ← Módulo RXD
// No hay pin DE — el módulo conmuta TX/RX automáticamente

// ── Configuración Modbus ──────────────────────────────────────────────────────
#define ORBIS_ID     190    // *** Cambia si tu contador tiene otra dirección ***
#define BAUDRATE    9600
#define TIMEOUT_MS  1000

// ── Registros ORBIS CONTAX D-6041-BUS (Input Registers, función 0x04) ────────
//   0x0046 (70)   Voltaje        U16  Gain=10    V
//   0x004C (76)   Corriente      U16  Gain=100   A
//   0x004F (79)   Potencia       U16  Gain=100   kW
//   0x2110 (8464) Energía diaria U32  Gain=1000  kWh  (2 registros)

// ── Debug: true = muestra tramas hex + diagnóstico ────────────────────────────
#define DEBUG_MODE  true

// ── Intervalo entre ciclos de lectura ─────────────────────────────────────────
#define READ_INTERVAL_MS  3000

// =============================================================================
//  Variables de datos
// =============================================================================
float    ct_voltage_v  = 0.0f;
float    ct_current_a  = 0.0f;
float    ct_power_kw   = 0.0f;
float    ct_energy_kwh = 0.0f;

uint32_t lastReadTime  = 0;

// =============================================================================
//  Declaraciones anticipadas (forward declarations)
//  Necesarias en .cpp — en .ino el compilador las genera automáticamente
// =============================================================================
uint16_t crc16            (const uint8_t *data, uint8_t len);
void     printHex         (const uint8_t *buf,  uint8_t len);
void     sendModbusRequest(uint8_t slaveId, uint8_t funcCode,
                           uint16_t regAddr, uint16_t numRegs);
uint8_t  readModbusResponse(uint8_t *buf, uint8_t maxLen);
bool     validateResponse (const uint8_t *buf, uint8_t len,
                           uint8_t expectedId, uint8_t expectedFc);
float    readU16          (uint16_t regAddr, float gain);
float    readU32reg       (uint16_t regAddr, float gain);
void     readAllOrbis     ();
void     printSummary     ();

// =============================================================================
//  CRC-16 Modbus
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
//  printHex
// =============================================================================
void printHex(const uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
}

// =============================================================================
//  sendModbusRequest
//  Construye y envía la trama Modbus RTU por Serial2.
//  Sin control DE — el módulo conmuta solo.
// =============================================================================
void sendModbusRequest(uint8_t slaveId, uint8_t funcCode,
                       uint16_t regAddr, uint16_t numRegs) {
  // Vaciar buffer RX antes de enviar (descartar basura residual)
  while (Serial2.available()) Serial2.read();

  uint8_t frame[8];
  frame[0] = slaveId;
  frame[1] = funcCode;
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] =  regAddr       & 0xFF;
  frame[4] = (numRegs >> 8) & 0xFF;
  frame[5] =  numRegs       & 0xFF;

  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;          // CRC byte bajo (primero en Modbus RTU)
  frame[7] = (crc >> 8) & 0xFF;  // CRC byte alto

  if (DEBUG_MODE) {
    Serial.print(F("  [TX] "));
    printHex(frame, 8);
    Serial.println();
  }

  Serial2.write(frame, 8);
  Serial2.flush();  // Espera a que se transmitan todos los bytes antes de
                    // intentar recibir (el módulo aún está en TX hasta aquí)
}

// =============================================================================
//  readModbusResponse
//  Espera la respuesta del esclavo.
//  Devuelve número de bytes recibidos (0 = timeout).
// =============================================================================
uint8_t readModbusResponse(uint8_t *buf, uint8_t maxLen) {
  uint32_t t0  = millis();
  uint8_t  idx = 0;

  // Esperar primer byte
  while (!Serial2.available()) {
    if (millis() - t0 > TIMEOUT_MS) {
      if (DEBUG_MODE) Serial.println(F("  [RX] TIMEOUT — sin respuesta"));
      return 0;
    }
  }

  // Leer hasta silencio de 4 ms (= fin de trama RTU a 9600 bps)
  t0 = millis();
  while (millis() - t0 < TIMEOUT_MS) {
    if (Serial2.available()) {
      if (idx < maxLen) buf[idx++] = Serial2.read();
      else              Serial2.read();
      t0 = millis();
    }
    if (idx > 0 && !Serial2.available() && (millis() - t0 > 4)) break;
  }

  if (DEBUG_MODE && idx > 0) {
    Serial.print(F("  [RX] "));
    printHex(buf, idx);
    Serial.print(F("  (")); Serial.print(idx); Serial.println(F(" bytes)"));
  }

  return idx;
}

// =============================================================================
//  validateResponse
// =============================================================================
bool validateResponse(const uint8_t *buf, uint8_t len,
                      uint8_t expectedId, uint8_t expectedFc) {
  if (len < 5) {
    Serial.print(F("  [ERR] Respuesta corta: "));
    Serial.print(len); Serial.println(F(" bytes"));
    return false;
  }

  uint16_t crcCalc = crc16(buf, len - 2);
  uint16_t crcRecv = (uint16_t)buf[len-2] | ((uint16_t)buf[len-1] << 8);
  if (crcCalc != crcRecv) {
    Serial.print(F("  [ERR] CRC — calculado: 0x")); Serial.print(crcCalc, HEX);
    Serial.print(F("  recibido: 0x"));              Serial.println(crcRecv, HEX);
    return false;
  }

  if (buf[0] != expectedId) {
    Serial.print(F("  [ERR] ID inesperado: "));
    Serial.print(buf[0]); Serial.print(F(" (esperado ")); Serial.print(expectedId); Serial.println(')');
    return false;
  }

  if (buf[1] & 0x80) {
    Serial.print(F("  [ERR] Excepcion Modbus: 0x")); Serial.println(buf[2], HEX);
    // 0x01=funcion ilegal  0x02=dir.ilegal  0x03=valor ilegal  0x04=fallo esclavo
    return false;
  }

  if (buf[1] != expectedFc) {
    Serial.print(F("  [ERR] FC inesperado: 0x")); Serial.print(buf[1], HEX);
    Serial.print(F(" (esperado 0x"));              Serial.print(expectedFc, HEX); Serial.println(')');
    return false;
  }

  return true;
}

// =============================================================================
//  readU16 / readU32reg — lectura de registros Input (0x04)
// =============================================================================
float readU16(uint16_t regAddr, float gain) {
  uint8_t buf[16];
  sendModbusRequest(ORBIS_ID, 0x04, regAddr, 1);
  uint8_t len = readModbusResponse(buf, sizeof(buf));

  if (!validateResponse(buf, len, ORBIS_ID, 0x04)) return NAN;
  if (buf[2] != 2) { Serial.println(F("  [ERR] Bytes inesperados U16")); return NAN; }

  uint16_t raw = ((uint16_t)buf[3] << 8) | buf[4];
  return (float)raw / gain;
}

float readU32reg(uint16_t regAddr, float gain) {
  uint8_t buf[16];
  sendModbusRequest(ORBIS_ID, 0x04, regAddr, 2);
  uint8_t len = readModbusResponse(buf, sizeof(buf));

  if (!validateResponse(buf, len, ORBIS_ID, 0x04)) return NAN;
  if (buf[2] != 4) { Serial.println(F("  [ERR] Bytes inesperados U32")); return NAN; }

  uint32_t raw = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16)
               | ((uint32_t)buf[5] <<  8) |  (uint32_t)buf[6];
  return (float)raw / gain;
}

// =============================================================================
//  readAllOrbis
// =============================================================================
void readAllOrbis() {
  Serial.println(F("\n--- Leyendo ORBIS CONTAX D-6041-BUS ---"));

  Serial.println(F("[1/4] Potencia (0x004F)"));
  float p = readU16(0x004F, 100.0f);
  if (!isnan(p)) { ct_power_kw = p;
    Serial.print(F("      -> ")); Serial.print(ct_power_kw, 3); Serial.println(F(" kW")); }
  delay(100);

  Serial.println(F("[2/4] Corriente (0x004C)"));
  float i = readU16(0x004C, 100.0f);
  if (!isnan(i)) { ct_current_a = i;
    Serial.print(F("      -> ")); Serial.print(ct_current_a, 2); Serial.println(F(" A")); }
  delay(100);

  Serial.println(F("[3/4] Voltaje (0x0046)"));
  float v = readU16(0x0046, 10.0f);
  if (!isnan(v)) { ct_voltage_v = v;
    Serial.print(F("      -> ")); Serial.print(ct_voltage_v, 1); Serial.println(F(" V")); }
  delay(100);

  Serial.println(F("[4/4] Energia diaria (0x2110)"));
  float e = readU32reg(0x2110, 1000.0f);
  if (!isnan(e)) { ct_energy_kwh = e;
    Serial.print(F("      -> ")); Serial.print(ct_energy_kwh, 3); Serial.println(F(" kWh")); }
}

// =============================================================================
//  printSummary — resumen + línea para Serial Plotter
// =============================================================================
void printSummary() {
  Serial.println(F("\n========================================"));
  Serial.print  (F("  Voltaje   : ")); Serial.print(ct_voltage_v,  1); Serial.println(F(" V"));
  Serial.print  (F("  Corriente : ")); Serial.print(ct_current_a,  2); Serial.println(F(" A"));
  Serial.print  (F("  Potencia  : ")); Serial.print(ct_power_kw,   3); Serial.println(F(" kW"));
  Serial.print  (F("  Energia   : ")); Serial.print(ct_energy_kwh, 3); Serial.println(F(" kWh"));
  Serial.println(F("========================================"));

  // Línea para Serial Plotter (PlatformIO: Monitor → Serial Plotter)
  Serial.print(F("Potencia_kW:")); Serial.print(ct_power_kw,   3); Serial.print(',');
  Serial.print(F("Corriente_A:")); Serial.print(ct_current_a,  2); Serial.print(',');
  Serial.print(F("Voltaje_V:"));   Serial.println(ct_voltage_v, 1);
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("\n============================================"));
  Serial.println(F("  TEST Modbus RTU — ESP32 DevKit V1"));
  Serial.println(F("  Modulo MAX3485 rojo (sin pin DE)"));
  Serial.println(F("  Contador ORBIS CONTAX D-6041-BUS"));
  Serial.println(F("============================================"));
  Serial.println(F("  TX GPIO17 -> TXD   RX GPIO16 <- RXD"));
  Serial.println(F("  Jumper '20 S' debe estar QUITADO"));
  Serial.print  (F("  ID Modbus : ")); Serial.println(ORBIS_ID);
  Serial.print  (F("  Baudrate  : ")); Serial.println(BAUDRATE);
  Serial.println(F("============================================\n"));

  // Serial2: UART2 del ESP32, TX=GPIO17, RX=GPIO16, 8N1
  Serial2.begin(BAUDRATE, SERIAL_8N1, RS485_RX, RS485_TX);

  delay(500);  // Estabilización del bus

  Serial.println(F(">>> Iniciando lecturas <<<\n"));
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = millis();
    readAllOrbis();
    printSummary();
  }
}