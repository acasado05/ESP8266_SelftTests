#include <Arduino.h>

#define DEBUG_MODE  true

// ── Pines de hardware (Asegúrate de mantener el divisor de tensión en RX) ───
#define RS485_TX    17    // GPIO17 -> Módulo DI
#define RS485_RX    16    // GPIO16 <- Módulo RO
#define RE_DE_PIN   4     // GPIO4  -> Control RE_DE

// ── Configuración Huawei SUN2000 ───────────────────────────────────────────
#define HUAWEI_ID    1      // Por defecto los inversores Huawei usan el ID 1
#define BAUDRATE     9600   //
#define TIMEOUT_MS   1500   // Huawei a veces tarda un poco más en responder

// ── Registros Clave (Función 0x03 - Holding Registers) ─────────────────────
// Nota: Algunos mapas de registros requieren restar 1 a la dirección
#define REG_PV2_VOLTAGE   32018   // U16, Gain 10 (V)
#define REG_PV2_CURRENT   32019   // I16, Gain 100 (A)
#define REG_P_INPUT_DC    32064   // I32, Gain 1 (W)
#define REG_P_ACTIVE_AC   32080   // I32, Gain 1 (W)
#define REG_E_DIARIA      32114   // U32, Gain 100 (kWh)

// ── Prototipos ──────────────────────────────────────────────────────────────
uint16_t crc16(const uint8_t *data, uint8_t len);
void sendModbusRequest(uint8_t slaveId, uint8_t funcCode, uint16_t regAddr, uint16_t numRegs);
uint8_t readModbusResponse(uint8_t *buf, uint8_t maxLen);
void readHuaweiInverter();
bool validateResponse(const uint8_t *buf, uint8_t len, uint8_t expectedId, uint8_t expectedFc);
void printHex(const uint8_t *buf, uint8_t len);
void logHuaweiFormat();


// ── Variables globales ──────────────────────────────────────────────────────
float v_pv2 = 0.0, i_pv2 = 0.0;
int32_t p_dc_in = 0, p_ac_out = 0;
float e_hoy = 0.0;
uint32_t lastMillis = 0;

void setup() {
    Serial.begin(115200);
    pinMode(RE_DE_PIN, OUTPUT);
    digitalWrite(RE_DE_PIN, LOW); // Modo recepción inicial

    // Inicializar UART2 para RS-485
    Serial2.begin(BAUDRATE, SERIAL_8N1, RS485_RX, RS485_TX);
    Serial.println(F("Iniciando comunicación con Huawei SUN2000..."));
}

void loop() {
    if (millis() - lastMillis > 5000) { // Lectura cada 5 segundos
        lastMillis = millis();
        readHuaweiInverter();
        logHuaweiFormat();
    }
}

// =============================================================================
//  Gestión de Envío con Control de Flujo (GPIO4)
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

// =============================================================================
//  Lectura de Datos del Inversor
// =============================================================================
void readHuaweiInverter() {
  uint8_t res[64];

  // 1. Leer String 2 (Tensión y Corriente) - 2 registros consecutivos
  sendModbusRequest(HUAWEI_ID, 0x03, REG_PV2_VOLTAGE, 2);
  if (readModbusResponse(res, sizeof(res)) >= 7) {
    v_pv2 = (float)((uint16_t)res[3] << 8 | res[4]) / 10.0f;
    i_pv2 = (float)((int16_t)res[5] << 8 | res[6]) / 100.0f;
  }
  delay(100);

  // 2. Leer Potencia Entrada DC (I32)
  sendModbusRequest(HUAWEI_ID, 0x03, REG_P_INPUT_DC, 2);
  if (readModbusResponse(res, sizeof(res)) >= 9) {
    p_dc_in = (int32_t)res[3] << 24 | (int32_t)res[4] << 16 | (int32_t)res[5] << 8 | (int32_t)res[6];
  }
  delay(100);

  // 3. Leer Potencia Salida AC (I32)
  sendModbusRequest(HUAWEI_ID, 0x03, REG_P_ACTIVE_AC, 2);
  if (readModbusResponse(res, sizeof(res)) >= 9) {
    p_ac_out = (int32_t)res[3] << 24 | (int32_t)res[4] << 16 | (int32_t)res[5] << 8 | (int32_t)res[6];
  }
}

/**
 * Calcula el CRC-16 para Modbus RTU.
 * @param data Puntero al array de datos.
 * @param len Longitud de los datos.
 * @return Valor del CRC-16 calculado. [cite: 8, 12]
 */
uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF; // Inicialización del registro CRC [cite: 8]
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i]; // Operación XOR con el byte de datos [cite: 9]
    for (uint8_t b = 0; b < 8; b++) {
      // Desplazamiento y aplicación del polinomio 0xA001 si el bit LSB es 1 [cite: 10, 11]
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

/**
 * Lee la respuesta del bus serie aplicando un filtro para el ruido inicial.
 * @param buf Buffer donde se almacenará la respuesta.
 * @param maxLen Capacidad máxima del buffer.
 * @return Número de bytes leídos. [cite: 22, 23]
 */
uint8_t readModbusResponse(uint8_t *buf, uint8_t maxLen) {
  uint32_t t0 = millis();
  uint8_t idx = 0;
  
  // Esperar a que el primer byte esté disponible o se agote el tiempo [cite: 24]
  while (!Serial2.available()) {
    if (millis() - t0 > TIMEOUT_MS) {
      if (DEBUG_MODE) Serial.println(F("  [RX] TIMEOUT — sin respuesta"));
      return 0; // Retorna 0 si hay timeout [cite: 25]
    }
  }

  t0 = millis();
  // Lectura de la trama completa [cite: 26]
  while (millis() - t0 < TIMEOUT_MS) {
    if (Serial2.available()) {
      uint8_t c = Serial2.read();
      
      // FILTRO DE RUIDO: Ignoramos el byte 0x00 si es el primero de la trama
      if (idx == 0 && c == 0x00) continue; 

      if (idx < maxLen) buf[idx++] = c;
      t0 = millis(); // Reiniciar contador de tiempo tras recibir byte [cite: 28]
    }
    // Si hay un silencio de más de 4ms, se considera fin de trama Modbus [cite: 29]
    if (idx > 0 && !Serial2.available() && (millis() - t0 > 4)) break;
  }

  if (DEBUG_MODE && idx > 0) {
    Serial.print(F("  [RX-Limpia] "));
    printHex(buf, idx);
    Serial.printf(" (%d bytes)\n", idx);
  }
  return idx;
}

/**
 * Valida la integridad de la trama recibida (ID, Código de función y CRC). [cite: 31]
 */
bool validateResponse(const uint8_t *buf, uint8_t len, uint8_t expectedId, uint8_t expectedFc) {
  if (len < 5) return false; // Una trama Modbus válida tiene al menos 5 bytes [cite: 32]

  // Verificar el CRC de la trama recibida [cite: 33]
  uint16_t crcCalc = crc16(buf, len - 2);
  uint16_t crcRecv = (uint16_t)buf[len-2] | ((uint16_t)buf[len-1] << 8);
  if (crcCalc != crcRecv) {
    if (DEBUG_MODE) Serial.println(F("  [ERR] Error de CRC"));
    return false;
  }

  // Comprobar que el ID del esclavo coincide [cite: 35]
  if (buf[0] != expectedId) return false;

  // Comprobar si hay una excepción Modbus (bit MSB del código de función) [cite: 36]
  if (buf[1] & 0x80) {
    if (DEBUG_MODE) Serial.printf("  [ERR] Excepción Modbus: 0x%02X\n", buf[2]);
    return false;
  }

  // Verificar que el código de función es el esperado [cite: 38]
  if (buf[1] != expectedFc) return false;

  return true;
}

/**
 * Imprime un buffer en formato hexadecimal para depuración. [cite: 13]
 */
void printHex(const uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
}

void logHuaweiFormat() {
  Serial.println(F("\n========================================"));
  Serial.println(F("    TELEMETRÍA INVERSOR HUAWEI L1       "));
  Serial.println(F("----------------------------------------"));
  
  // Usamos printf para alinear columnas perfectamente
  Serial.printf("%-18s : %7.2f V\n", "Tensión PV2", v_pv2);
  Serial.printf("%-18s : %7.2f A\n", "Corriente PV2", i_pv2);
  Serial.printf("%-18s : %7d W\n",   "Potencia DC (In)", p_dc_in);
  Serial.printf("%-18s : %7d W\n",   "Potencia AC (Out)", p_ac_out);
  
  // Cálculo de eficiencia instantánea (opcional para tu TFG)
  if (p_dc_in > 0) {
    float eff = ((float)p_ac_out / (float)p_dc_in) * 100.0f;
    Serial.printf("%-18s : %7.1f %%\n", "Eficiencia Inv.", eff);
  }
  
  Serial.println(F("========================================\n"));
}