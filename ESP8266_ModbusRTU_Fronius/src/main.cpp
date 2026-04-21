/**
 * ============================================================
 *  Fronius Primo 3.0 — Lectura Modbus RTU con ESP8266 + MAX3485
 * ============================================================
 *
 *  TFG — Sistema embebido inteligente con Edge AI / TinyML
 *  Plataforma de prueba modular: ESP8266 + MAX3485
 *  Plataforma final:             ESP32-S3-DevKitC-1 + MAX3485
 *
 *  Protocolo: Modbus RTU sobre RS485
 *  Mapa de registros: Fronius Datamanager 2.0
 *                     SunSpec Float Model (ID 111 — monofásico)
 *
 *  Registros leídos (todos float32, 2 registros de 16-bit c/u):
 *    40072-40073  A     — Corriente AC total      [A]
 *    40086-40087  PhVphA — Tensión AC fase-neutro  [V]
 *    40092-40093  W     — Potencia AC activa       [W]
 *    40104-40105  DCA   — Corriente DC             [A]
 *    40106-40107  DCV   — Tensión DC               [V]
 *    40108-40109  DCW   — Potencia DC total         [W]
 *    40118        St    — Estado operativo          (enum)
 *
 *  IMPORTANTE — Direccionamiento Modbus:
 *    El protocolo Modbus usa direcciones con base-0, mientras que
 *    el mapa de Fronius las lista con base-1 (40001 = reg 0 en el
 *    frame). La fórmula es:  addr_modbus = addr_fronius - 1
 *    Ejemplo: registro 40072 → se pide dirección 40071 (0x9C57)
 *
 *  Configuración Modbus:
 *    - Baudrate:   9600 bps (por defecto en Fronius Datamanager 2.0)
 *    - Data bits:  8
 *    - Paridad:    None
 *    - Stop bits:  1  (8N1)
 *    - Modbus ID:  1  (inverter number 1 en menú DATCOM)
 *    - Función:    0x03 (Read Holding Registers)
 *
 *  Conexionado MAX3485 (módulo auto-direction) ↔ ESP8266:
 *    MAX3485 PIN    →  ESP8266 PIN
 *    ------------------------------------
 *    VCC            →  3.3 V
 *    GND            →  GND
 *    TXD / DI       →  GPIO1 / TX (UART0)
 *    RXD / RO       →  GPIO3 / RX (UART0)
 *    A (RS485+)     →  Hilo D+ del bus Fronius (naranja)
 *    B (RS485-)     →  Hilo D- del bus Fronius (blanco)
 *
 *  NOTA módulo auto-direction:
 *    DE y RE están unidos internamente y conectados a la línea DI.
 *    El módulo activa TX automáticamente cuando DI tiene datos, y
 *    vuelve a RX en reposo. NO se necesita pin de control externo.
 *    El GND del bus RS485 del Fronius debe conectarse al GND del
 *    ESP8266 para tener referencia común entre ambos equipos.
 *
 *  Conexionado MAX3485 ↔ Fronius Datamanager 2.0:
 *    El Fronius expone RS485 en el conector Modbus del Datamanager.
 *    El hilo naranja del kit Fronius = D+ → MAX3485 A
 *    El hilo blanco  del kit Fronius = D- → MAX3485 B
 *    Activar resistencia de terminación (120 Ω) en el Datamanager
 *    que esté al final físico del bus (switch ON en la PCB).
 *
 *  Debug en desarrollo:
 *    UART0 (Serial) es compartido con el bus Modbus, por lo que los
 *    prints interferirán con el tráfico RS485. Se usa Serial1 (GPIO2,
 *    TX-only) a 115200 bps exclusivamente para depuración. Conectar
 *    un adaptador USB-Serial al GPIO2 para ver los mensajes de debug.
 *
 *  platformio.ini (ejemplo):
 *    [env:esp8266]
 *    platform  = espressif8266
 *    board     = nodemcuv2
 *    framework = arduino
 *    monitor_speed = 115200
 *
 * ============================================================
 */

#include <Arduino.h>

// ── Debug por Serial1 (GPIO2, TX-only, 115200 bps) ───────────
// Serial1 no interfiere con el bus Modbus en UART0.
// Conectar un adaptador USB-Serial al GPIO2 para ver los mensajes.
// Para deshabilitar el debug basta con comentar esta línea:
#define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
  #define DBG_BEGIN()   Serial1.begin(115200)
  #define DBG(...)        Serial1.print(__VA_ARGS__)
  #define DBGLN(...)      Serial1.println(__VA_ARGS__)
  #define DBGHEX(x)     Serial1.print(x, HEX)
#else
  #define DBG_BEGIN()
  #define DBG(x)
  #define DBGLN(x)
  #define DBGHEX(x)
#endif

// ── Configuración Modbus ──────────────────────────────────────
#define MODBUS_BAUD    9600
#define MODBUS_ID      01    // Dirección esclavo del Fronius (ajustar según menú DATCOM)

// ── Tiempos ───────────────────────────────────────────────────
// t3.5 = 3.5 tramas de silencio entre mensajes.
// A 9600 bps, 1 carácter = 1.042 ms → 3.5 chars ≈ 3.65 ms → usamos 4 ms.
// Tiempo de espera máximo para recibir respuesta completa.
#define MODBUS_T35_US       4000   // µs de silencio inter-frame
#define MODBUS_TIMEOUT_MS   500    // ms de espera máxima de respuesta
#define POLL_INTERVAL_MS    2000   // ms entre lecturas al inversor

// ── Registro base del modelo SunSpec Float (monofásico, ID 111) ─
// Fronius: registro 40001 → dirección Modbus 0x9C40 (= 40000 en base-0).
// Todos los registros del mapa se piden con addr = fronius_reg - 1.
//
// Bloque de interés (registros Fronius → dirección Modbus en base-0):
//   40072 → 0x9C57  A      (AC Current)       float32 [2 regs]
//   40086 → 0x9C65  PhVphA (AC Voltage A-N)   float32 [2 regs]
//   40092 → 0x9C6B  W      (AC Power)         float32 [2 regs]
//   40104 → 0x9C77  DCA    (DC Current)       float32 [2 regs]
//   40106 → 0x9C79  DCV    (DC Voltage)       float32 [2 regs]
//   40108 → 0x9C7B  DCW    (DC Power)         float32 [2 regs]
//   40118 → 0x9C85  St     (Operating State)  uint16  [1 reg ]
//
// Leemos un único bloque contiguo: desde 40072 hasta 40118 inclusive
// = 47 registros (0x2F). Así minimizamos las transacciones Modbus.

#define REG_START_FRONIUS   40072   // primer registro del bloque (base-1)
#define REG_COUNT           47      // número de registros a leer

// Offsets dentro del buffer de respuesta (cada float32 ocupa 2 words)
// offset = (fronius_reg - REG_START_FRONIUS)
#define OFF_AC_CURRENT   0    // 40072-40073  A
#define OFF_AC_VOLTAGE  14    // 40086-40087  PhVphA
#define OFF_AC_POWER    20    // 40092-40093  W
#define OFF_DC_CURRENT  32    // 40104-40105  DCA
#define OFF_DC_VOLTAGE  34    // 40106-40107  DCV
#define OFF_DC_POWER    36    // 40108-40109  DCW
#define OFF_STATE       46    // 40118        St  (uint16, 1 reg)

// ── Nombres de estado operativo (SunSpec enum16) ──────────────
const char* stateNames[] = {
    "OFF",           // 1
    "SLEEPING",      // 2
    "STARTING",      // 3
    "MPPT",          // 4  ← normal operation
    "THROTTLED",     // 5
    "SHUTTING_DOWN", // 6
    "FAULT",         // 7
    "STANDBY"        // 8
};

// ── Prototipos ────────────────────────────────────────────────
uint16_t crc16(const uint8_t* buf, uint8_t len);
bool     sendModbusRequest(uint8_t slaveId, uint16_t startReg, uint16_t numRegs);
int      readModbusResponse(uint8_t* buf, uint16_t maxLen);
float    regsToFloat(const uint8_t* buf, uint16_t byteOffset);
uint16_t regsToUint16(const uint8_t* buf, uint16_t byteOffset);
void     printResults(float acV, float acA, float acW,
                      float dcV, float dcA, float dcW,
                      uint16_t state);

// ── Buffers globales ──────────────────────────────────────────
static uint8_t rxBuf[128];   // buffer de recepción Modbus

// =============================================================
void setup() {
    // UART0 (Serial) → bus Modbus RTU exclusivamente (9600 8N1)
    // UART1 (Serial1, GPIO2 TX-only) → debug a 115200 bps
    Serial.begin(MODBUS_BAUD, SERIAL_8N1);
    DBG_BEGIN();

    // Pequeña pausa para estabilizar la UART y el MAX3485
    delay(200);

    DBGLN("\n--- Fronius Primo Modbus RTU Reader ---");
    DBG  ("Slave ID  : "); DBGLN(MODBUS_ID);
    DBG  ("Start reg : "); DBGLN(REG_START_FRONIUS);
    DBG  ("Num regs  : "); DBGLN(REG_COUNT);
    DBGLN("Modulo MAX3485 auto-direction (sin pin DE/RE)");
    DBGLN("---------------------------------------\n");
    delay(200);
}

// =============================================================
void loop() {
    static uint32_t lastPoll = 0;
    uint32_t now = millis();

    if (now - lastPoll < POLL_INTERVAL_MS) return;
    lastPoll = now;

    // 1) Enviar petición Modbus FC03
    if (!sendModbusRequest(MODBUS_ID,
                           REG_START_FRONIUS - 1,  // base-0
                           REG_COUNT)) {
        DBGLN("[ERROR] Fallo al enviar peticion Modbus");
        return;
    }

    // 2) Esperar y leer respuesta
    int rxLen = readModbusResponse(rxBuf, sizeof(rxBuf));

    if (rxLen < 0) {
        DBGLN("[ERROR] Timeout esperando respuesta del inversor");
        return;
    }

    // 3) Validar respuesta mínima
    // Estructura: [SlaveID][FC][ByteCount][Data...][CRC_L][CRC_H]
    // ByteCount = REG_COUNT * 2 bytes
    uint8_t expectedBytes = REG_COUNT * 2;
    if (rxLen < (3 + expectedBytes + 2)) {
        DBG("[ERROR] Respuesta demasiado corta: ");
        DBG(rxLen); DBGLN(" bytes");
        return;
    }

    // 4) Verificar CRC de la respuesta
    uint16_t crcCalc = crc16(rxBuf, rxLen - 2);
    uint16_t crcRecv = (uint16_t)rxBuf[rxLen - 1] << 8 |
                       (uint16_t)rxBuf[rxLen - 2];
    if (crcCalc != crcRecv) {
        DBGLN("[ERROR] CRC incorrecto en respuesta");
        return;
    }

    // 5) Verificar ID y función
    if (rxBuf[0] != MODBUS_ID) {
        DBGLN("[ERROR] Slave ID inesperado en respuesta");
        return;
    }
    if (rxBuf[1] == 0x83) {
        // El esclavo devolvió excepción Modbus
        DBG("[ERROR] Excepcion Modbus, codigo: 0x");
        DBGLN(rxBuf[2], HEX);
        return;
    }
    if (rxBuf[1] != 0x03) {
        DBGLN("[ERROR] Funcion Modbus inesperada en respuesta");
        return;
    }

    // 6) Datos empiezan en byte 3 (índice 3 del buffer)
    const uint8_t* data = &rxBuf[3];

    // 7) Decodificar valores
    // Cada float32 ocupa 4 bytes (2 registros × 2 bytes/registro).
    // El Fronius usa big-endian (byte más significativo primero),
    // igual que el estándar IEEE-754 sobre Modbus.
    float acCurrent = regsToFloat(data, OFF_AC_CURRENT * 2);
    float acVoltage = regsToFloat(data, OFF_AC_VOLTAGE * 2);
    float acPower   = regsToFloat(data, OFF_AC_POWER   * 2);
    float dcCurrent = regsToFloat(data, OFF_DC_CURRENT * 2);
    float dcVoltage = regsToFloat(data, OFF_DC_VOLTAGE * 2);
    float dcPower   = regsToFloat(data, OFF_DC_POWER   * 2);
    uint16_t state  = regsToUint16(data, OFF_STATE     * 2);

    // 8) Mostrar resultados
    printResults(acVoltage, acCurrent, acPower,
                 dcVoltage, dcCurrent, dcPower, state);
}

// =============================================================
//  CRC-16/IBM (Modbus CRC): polinomio 0xA001
// =============================================================
uint16_t crc16(const uint8_t* buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// =============================================================
//  Construye y envía una petición Modbus FC03
//  startReg: dirección base-0 del primer registro
//  numRegs:  número de registros a leer
// =============================================================
bool sendModbusRequest(uint8_t slaveId, uint16_t startReg, uint16_t numRegs) {
    uint8_t req[8];
    req[0] = slaveId;
    req[1] = 0x03;                    // FC03: Read Holding Registers
    req[2] = (startReg >> 8) & 0xFF;  // Registro alto
    req[3] =  startReg & 0xFF;        // Registro bajo
    req[4] = (numRegs  >> 8) & 0xFF;  // Cantidad alta
    req[5] =  numRegs  & 0xFF;        // Cantidad baja
    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;              // CRC bajo (primero en Modbus)
    req[7] = (crc >> 8) & 0xFF;       // CRC alto

    // Limpiar buffer de entrada antes de transmitir
    while (Serial.available()) Serial.read();

    // El módulo auto-direction activa TX automáticamente al escribir.
    // No se necesita control manual de DE/RE.
    Serial.write(req, 8);
    Serial.flush();  // esperar a que se vacíe el buffer TX por hardware

    // Guardar silencio t3.5 para que el módulo vuelva a modo RX
    // y el esclavo reconozca el fin de trama antes de responder.
    delayMicroseconds(MODBUS_T35_US);

    return true;
}

// =============================================================
//  Lee la respuesta Modbus del esclavo
//  Retorna número de bytes leídos, o -1 en timeout
// =============================================================
int readModbusResponse(uint8_t* buf, uint16_t maxLen) {
    uint16_t idx = 0;
    uint32_t start = millis();
    uint32_t lastByte = start;

    while (true) {
        // Timeout global
        if ((millis() - start) > MODBUS_TIMEOUT_MS) {
            return (idx > 0) ? (int)idx : -1;
        }

        if (Serial.available()) {
            if (idx < maxLen) {
                buf[idx++] = (uint8_t)Serial.read();
                lastByte = millis();
            } else {
                Serial.read();  // descartar si el buffer está lleno
            }
        } else {
            // Detectar fin de trama: silencio > t3.5 DESPUÉS de haber
            // recibido al menos el header (3 bytes mínimo)
            if (idx >= 3) {
                uint32_t elapsed = millis() - lastByte;
                // A 9600 bps t3.5 ≈ 4 ms
                if (elapsed > 5) break;
            }
        }
    }
    return (int)idx;
}

// =============================================================
//  Convierte 4 bytes big-endian a float32 (IEEE-754)
//  byteOffset: posición en el buffer de datos (sin header Modbus)
// =============================================================
float regsToFloat(const uint8_t* buf, uint16_t byteOffset) {
    // Modbus entrega los bytes en big-endian:
    // buf[off+0] = byte más significativo de la palabra alta
    // buf[off+1] = byte menos significativo de la palabra alta
    // buf[off+2] = byte más significativo de la palabra baja
    // buf[off+3] = byte menos significativo de la palabra baja
    // Esto coincide directamente con IEEE-754 big-endian.
    uint32_t raw = ((uint32_t)buf[byteOffset    ] << 24) |
                   ((uint32_t)buf[byteOffset + 1] << 16) |
                   ((uint32_t)buf[byteOffset + 2] <<  8) |
                   ((uint32_t)buf[byteOffset + 3]);
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

// =============================================================
//  Convierte 2 bytes big-endian a uint16
// =============================================================
uint16_t regsToUint16(const uint8_t* buf, uint16_t byteOffset) {
    return ((uint16_t)buf[byteOffset] << 8) | buf[byteOffset + 1];
}

// =============================================================
//  Imprime los resultados en formato legible
// =============================================================
void printResults(float acV, float acA, float acW,
                  float dcV, float dcA, float dcW,
                  uint16_t state) {
    const char* stateName = "UNKNOWN";
    if (state >= 1 && state <= 8) {
        stateName = stateNames[state - 1];
    }

    DBGLN("========================================");
    DBG  ("[Estado] "); DBG(state);
    DBG  (" - ");       DBGLN(stateName);
    DBGLN("--- AC ---");
    DBG  ("  Tension  : "); DBG(acV, 2); DBGLN(" V");
    DBG  ("  Corriente: "); DBG(acA, 3); DBGLN(" A");
    DBG  ("  Potencia : "); DBG(acW, 1); DBGLN(" W");
    DBGLN("--- DC ---");
    DBG  ("  Tension  : "); DBG(dcV, 2); DBGLN(" V");
    DBG  ("  Corriente: "); DBG(dcA, 3); DBGLN(" A");
    DBG  ("  Potencia : "); DBG(dcW, 1); DBGLN(" W");
    DBGLN("========================================\n");
}