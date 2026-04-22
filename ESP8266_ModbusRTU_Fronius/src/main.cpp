/**
 * ============================================================
 *  Fronius Primo 3.0 — Modbus RTU + OLED  (ESP8266 + MAX3485)
 * ============================================================
 *
 *  TFG — Sistema embebido inteligente con Edge AI / TinyML
 *  Plataforma de prueba: ESP8266 NodeMCU + MAX3485 auto-direction
 *                        + OLED SSD1306 128x64 I2C
 *
 * ── Por que se reescribio el codigo anterior ────────────────
 *
 *  El codigo anterior construia frames Modbus RTU a mano.
 *  El frame era correcto (visible en PulseView) pero el inversor
 *  no respondia. Al contrastar con el TFG de referencia y el
 *  mapa de registros Fronius se identificaron tres causas raiz:
 *
 *  1. REGISTROS INCORRECTOS PARA PRIMO CON 2 MPPTs.
 *     El mapa advierte que DCA/DCV del modelo base (40104-40107)
 *     "no estan soportados si hay multiples entradas DC". El
 *     Primo 3.0 tiene 2 strings MPPT, por lo que hay que usar
 *     el modelo Multiple MPPT (registros 40283-40305).
 *     El TFG de referencia confirma esto y usa exactamente esos
 *     registros con factor de escala x0.01 (sf = -2).
 *
 *  2. VALOR 0xFFFF durante arranque/parada.
 *     Documentado en TFG seccion 5.6: cuando el inversor esta
 *     encendiendo o apagando devuelve 0xFFFF en los registros.
 *     Se trata como 0 para evitar datos erroneos.
 *
 *  3. LIBRERIA ModbusMaster en lugar de frames manuales.
 *     La libreria 4-20ma/ModbusMaster gestiona internamente el
 *     timing inter-frame RTU, reintentos y validacion de CRC.
 *     Elimina los problemas de timing que causaban que el esclavo
 *     ignorara la peticion.
 *
 * ── Registros leidos ────────────────────────────────────────
 *
 *  Lectura 1 - Scale factors MPPT (FC03, 3 regs desde 40265):
 *    40266  DCA_SF  int16 (tipicamente -2)
 *    40267  DCV_SF  int16
 *    40268  DCW_SF  int16
 *
 *  Lectura 2 - Multiple MPPT String 1 (FC03, 3 regs desde 40282):
 *    40283  1_DCA  Corriente string 1  uint16  A  x 10^DCA_SF
 *    40284  1_DCV  Tension string 1    uint16  V  x 10^DCV_SF
 *    40285  1_DCW  Potencia string 1   uint16  W  x 10^DCW_SF
 *
 *  Lectura 3 - Multiple MPPT String 2 (FC03, 3 regs desde 40302):
 *    40303  2_DCA  Corriente string 2  uint16  A
 *    40304  2_DCV  Tension string 2    uint16  V
 *    40305  2_DCW  Potencia string 2   uint16  W
 *
 *  Lectura 4 - Bloque AC (FC03, 47 regs desde 40071):
 *    40072-40073  A      Corriente AC total  float32  A
 *    40086-40087  PhVphA Tension AC fase-N   float32  V
 *    40092-40093  W      Potencia AC activa  float32  W
 *    40118        St     Estado operativo    uint16   enum
 *
 *  Nota direccionamiento: las constantes REG_* ya estan en
 *  base-0 (= direccion mapa Fronius - 1). ModbusMaster las
 *  usa directamente.
 *
 * ── Configuracion Modbus ─────────────────────────────────────
 *    Baudrate  : 9600 bps, 8N1
 *    Modbus ID : 1  (probar 100 si no responde)
 *
 *    NOTA ID: verificar en pantalla del Fronius:
 *    SETUP -> DATCOM -> Wechselrichter-Nr.
 *    Si el numero mostrado es 0 -> Modbus ID = 100
 *    Si el numero mostrado es 1 -> Modbus ID = 1
 *
 * ── Conexionado ──────────────────────────────────────────────
 *    MAX3485 auto-direction:
 *      VCC -> 3.3V         GND -> GND
 *      TXD -> GPIO1/TX     RXD -> GPIO3/RX
 *      A   -> D+ Fronius (naranja)
 *      B   -> D- Fronius (blanco)
 *      GND bus Fronius -> GND ESP8266  (IMPRESCINDIBLE)
 *
 *    OLED SSD1306 I2C:
 *      SDA -> GPIO4 (D2)   SCL -> GPIO5 (D1)
 *      VCC -> 3.3V         GND -> GND
 *
 *    Debug Serial1 (TX-only):
 *      GPIO2 (D4) -> RX adaptador USB-Serial a 115200 bps
 *      Comentar #define DEBUG_ENABLED para deshabilitar.
 *
 * ── platformio.ini ───────────────────────────────────────────
 *    [env:esp8266]
 *    platform      = espressif8266
 *    board         = nodemcuv2
 *    framework     = arduino
 *    monitor_speed = 115200
 *    lib_deps =
 *      4-20ma/ModbusMaster @ ^2.0.1
 *      adafruit/Adafruit SSD1306 @ ^2.5.7
 *      adafruit/Adafruit GFX Library @ ^1.11.9
 *
 * ============================================================
 */

#include <Arduino.h>
#include <ModbusMaster.h>
#include <Wire.h>

// ── Configuracion Modbus ──────────────────────────────────────
#define MODBUS_ID          100        // Cambiar a 100 si no responde
#define PIN_RX2            16
#define PIN_TX2            17
#define MODBUS_BAUD        9600UL

// ── Timing ───────────────────────────────────────────────────
#define POLL_INTERVAL_MS   5000UL
#define MODBUS_TIMEOUT_MS  1000UL
#define MAX_RETRIES        3

// ── Registros Modbus (base-0 = mapa Fronius - 1) ─────────────
#define REG_MPPT_SF_BASE   40265    // DCA_SF, DCV_SF, DCW_SF
#define REG_MPPT1_BASE     40282    // I, V, P string 1
#define REG_MPPT2_BASE     40302    // I, V, P string 2
#define REG_AC_BASE        40071    // Bloque AC completo (47 regs)
#define REG_AC_COUNT       47

// Offsets en el buffer del bloque AC (en registros de 16-bit)
// offset = fronius_reg - 40072
#define OFF_AC_CURRENT      0       // 40072-40073  A       float32
#define OFF_AC_VOLTAGE     14       // 40086-40087  PhVphA  float32
#define OFF_AC_POWER       20       // 40092-40093  W       float32
#define OFF_STATE          46       // 40118        St      uint16

// ── OLED SSD1306 128x64 ───────────────────────────────────────
#define OLED_WIDTH         128
#define OLED_HEIGHT         64
#define OLED_ADDR          0x3C
#define OLED_RESET          -1

// ── Objetos globales ──────────────────────────────────────────
ModbusMaster modbus;

// ── Estructura de datos del inversor ─────────────────────────
struct InverterData {
    float    Idc1, Vdc1, Pdc1;    // DC string 1 [A, V, W]
    float    Idc2, Vdc2, Pdc2;    // DC string 2 [A, V, W]
    float    PdcTotal;             // Potencia DC total [W]
    float    Iac, Vac, Pac;        // AC [A, V, W]
    uint16_t state;                // Estado SunSpec enum16
    bool     valid;                // Ciclo de lectura exitoso
};

InverterData inv = {};

// Scale factors Multiple MPPT (int16, sunsssf)
// Valor por defecto -2 -> x0.01 (confirmado en TFG y mapa regs)
int16_t sf_DCA = -2;
int16_t sf_DCV = -2;
int16_t sf_DCW = -2;

// ── Prototipos ────────────────────────────────────────────────
float       applyScaleFactor(uint16_t raw, int16_t sf);
float       regsToFloat(uint16_t hi, uint16_t lo);
bool        readMPPTScaleFactors();
bool        readMPPTString(uint8_t str, float &I, float &V, float &P);
bool        readACBlock();
void        runReadCycle();
//void        updateOLED();
void        printDebug();
void        preTransmission();
void        postTransmission();
const char* stateStr(uint16_t st);

// =============================================================
void setup() {

    Serial.begin(115200); 

    // 1. Iniciamos UART2 (Modbus) con sus pines (16 y 17)
    Serial2.begin(MODBUS_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);

    // 2. Aplicamos el timeout a Serial2 (MUY IMPORTANTE)
    //Serial2.setTimeout(MODBUS_TIMEOUT_MS);

    // 3. Vinculamos la librería ModbusMaster a Serial2
    modbus.begin(MODBUS_ID, Serial2);

    modbus.preTransmission(preTransmission);
    modbus.postTransmission(postTransmission);

    // 4. UART0 (Serial) queda libre para ver mensajes en el PC  
    Serial.println("Comunicacion con Fronius iniciada en UART2...");
}

// =============================================================
void loop() {
    delay(100);
    static uint32_t lastPoll = 0;
    if (millis() - lastPoll < POLL_INTERVAL_MS) return;
    lastPoll = millis();

    runReadCycle();
    //updateOLED();
    printDebug();
}

void preTransmission() {
    // Flush cualquier byte residual antes de transmitir
    while (Serial2.available()) Serial2.read();
}

void postTransmission() {
    // Dar tiempo al módulo para volver a RX y vaciar el eco
    delay(2);
    while (Serial2.available()) Serial2.read();  // descartar eco propio
}

// =============================================================
//  Ciclo completo: SF -> MPPT1 -> MPPT2 -> Bloque AC
// =============================================================
void runReadCycle() {
    inv.valid = false;

    // Scale factors (si fallan, continua con valores por defecto)
    if (!readMPPTScaleFactors()) {
        Serial.println("[WARN] SF no leidos, usando sf=-2 (x0.01)");
    }

    // String 1 — obligatorio
    if (!readMPPTString(1, inv.Idc1, inv.Vdc1, inv.Pdc1)) {
        Serial.println("[ERR] Fallo MPPT1 — abortando ciclo");
        return;
    }

    // String 2 — no fatal (puede haber un solo string activo)
    if (!readMPPTString(2, inv.Idc2, inv.Vdc2, inv.Pdc2)) {
        Serial.println("[WARN] Fallo MPPT2 — asumiendo 0");
        inv.Idc2 = inv.Vdc2 = inv.Pdc2 = 0.0f;
    }

    inv.PdcTotal = inv.Pdc1 + inv.Pdc2;

    // Bloque AC — obligatorio
    if (!readACBlock()) {
        Serial.println("[ERR] Fallo bloque AC — abortando ciclo");
        return;
    }

    inv.valid = true;
}

// =============================================================
//  Scale factors del modelo Multiple MPPT
//  FC03, 3 registros desde 40265 (base-0)
// =============================================================
bool readMPPTScaleFactors() {
    for (int r = 0; r < MAX_RETRIES; r++) {
        uint8_t res = modbus.readHoldingRegisters(REG_MPPT_SF_BASE, 3);
        if (res == ModbusMaster::ku8MBSuccess) {
            sf_DCA = (int16_t)modbus.getResponseBuffer(0);
            sf_DCV = (int16_t)modbus.getResponseBuffer(1);
            sf_DCW = (int16_t)modbus.getResponseBuffer(2);
            Serial.printf("[SF] DCA=%d DCV=%d DCW=%d\n", sf_DCA, sf_DCV, sf_DCW);
            return true;
        }
        Serial.printf("[SF] retry %d err=0x%02X\n", r + 1, res);
        delay(150);
    }
    return false;
}

// =============================================================
//  Lectura de un string MPPT (I, V, P)
//  str: 1 o 2   |   FC03, 3 registros contiguos
// =============================================================
bool readMPPTString(uint8_t str, float &I, float &V, float &P) {
    uint16_t base = (str == 1) ? REG_MPPT1_BASE : REG_MPPT2_BASE;

    for (int r = 0; r < MAX_RETRIES; r++) {
        uint8_t res = modbus.readHoldingRegisters(base, 3);
        if (res == ModbusMaster::ku8MBSuccess) {
            uint16_t rawI = modbus.getResponseBuffer(0);
            uint16_t rawV = modbus.getResponseBuffer(1);
            uint16_t rawW = modbus.getResponseBuffer(2);

            // 0xFFFF = dato invalido (arranque/parada)
            // Documentado en TFG seccion 5.6 y mapa de registros
            I = (rawI == 0xFFFF) ? 0.0f : applyScaleFactor(rawI, sf_DCA);
            V = (rawV == 0xFFFF) ? 0.0f : applyScaleFactor(rawV, sf_DCV);
            P = (rawW == 0xFFFF) ? 0.0f : applyScaleFactor(rawW, sf_DCW);

            Serial.printf("[MPPT%d] I=%.3fA V=%.2fV P=%.1fW\n", str, I, V, P);
            return true;
        }
        Serial.printf("[MPPT%d] retry %d err=0x%02X\n", str, r + 1, res);
        delay(150);
    }
    return false;
}

// =============================================================
//  Bloque AC: 47 registros desde 40071 (base-0)
//  Decodifica float32 big-endian y el estado uint16
// =============================================================
bool readACBlock() {
    for (int r = 0; r < MAX_RETRIES; r++) {
        uint8_t res = modbus.readHoldingRegisters(REG_AC_BASE, REG_AC_COUNT);
        if (res == ModbusMaster::ku8MBSuccess) {
            // float32: word alto en offset N, word bajo en N+1
            inv.Iac   = regsToFloat(
                modbus.getResponseBuffer(OFF_AC_CURRENT),
                modbus.getResponseBuffer(OFF_AC_CURRENT + 1));
            inv.Vac   = regsToFloat(
                modbus.getResponseBuffer(OFF_AC_VOLTAGE),
                modbus.getResponseBuffer(OFF_AC_VOLTAGE + 1));
            inv.Pac   = regsToFloat(
                modbus.getResponseBuffer(OFF_AC_POWER),
                modbus.getResponseBuffer(OFF_AC_POWER + 1));
            inv.state = modbus.getResponseBuffer(OFF_STATE);

            Serial.printf("[AC] I=%.3fA V=%.2fV P=%.1fW St=%s\n",
                 inv.Iac, inv.Vac, inv.Pac, stateStr(inv.state));
            return true;
        }
        Serial.printf("[AC] retry %d err=0x%02X\n", r + 1, res);
        delay(150);
    }
    return false;
}

// =============================================================
//  Scale factor SunSpec: valor_fisico = raw * 10^sf
// =============================================================
float applyScaleFactor(uint16_t raw, int16_t sf) {
    float v = (float)raw;
    if (sf >= 0) {
        for (int i = 0; i < sf;  i++) v *= 10.0f;
    } else {
        for (int i = 0; i < -sf; i++) v *= 0.1f;
    }
    return v;
}

// =============================================================
//  Reconstruir float32 IEEE-754 big-endian desde dos uint16
// =============================================================
float regsToFloat(uint16_t hi, uint16_t lo) {
    uint32_t raw = ((uint32_t)hi << 16) | (uint32_t)lo;
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

// =============================================================
//  Nombre del estado SunSpec enum16
// =============================================================
const char* stateStr(uint16_t st) {
    switch (st) {
        case 1: return "OFF";
        case 2: return "SLEEPING";
        case 3: return "STARTING";
        case 4: return "MPPT";
        case 5: return "THROTTLE";
        case 6: return "SHUTTING";
        case 7: return "FAULT";
        case 8: return "STANDBY";
        default: return "UNKNOWN";
    }
}

// =============================================================
//  Resultados por Serial1 (debug)
// =============================================================
void printDebug() {
    if (!inv.valid) {
        Serial.println("[!] Error Modbus: El inversor no responde.");
        return;
    }

    Serial.println("\n+---------------------------------------+");
    Serial.printf("| FRONIUS ID: %-3d | ESTADO: %-10s |\n", MODBUS_ID, stateStr(inv.state));
    Serial.println("+---------------------------------------+");
    Serial.printf("| AC Power: %7.1f W | VAC: %5.1f V |\n", inv.Pac, inv.Vac);
    Serial.printf("| DC Total: %7.1f W | IAC: %5.2f A |\n", inv.PdcTotal, inv.Iac);
    Serial.println("+---------------------------------------+");
}