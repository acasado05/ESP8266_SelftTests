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
#include <SSD1306Wire.h>
#include <SoftwareSerial.h>

// ── Debug por Serial1 (GPIO2 TX-only, 115200 bps) ────────────
#define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
  #define DBG_BEGIN()  Serial1.begin(115200)
  #define DBG(x)       Serial1.print(x)
  #define DBGLN(x)     Serial1.println(x)
  #define DBGF(...)    Serial1.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN()
  #define DBG(x)
  #define DBGLN(x)
  #define DBGF(...)
#endif

// ── Configuracion Modbus ──────────────────────────────────────
#define MODBUS_ID          1        // Cambiar a 100 si no responde
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
SSD1306Wire display(0x3C, 12, 14);

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
void        updateOLED();
void        printDebug();
const char* stateStr(uint16_t st);

// =============================================================
void setup() {
    // UART0 exclusivo para Modbus RTU (9600 8N1)
    Serial.begin(MODBUS_BAUD, SERIAL_8N1);

    DBG_BEGIN();
    delay(300);

    // Inicializar ModbusMaster sobre UART0.
    // El modulo MAX3485 auto-direction no necesita callbacks
    // preTransmission/postTransmission; controla DE/RE solo.
    Serial.setTimeout(MODBUS_TIMEOUT_MS);
    modbus.begin(MODBUS_ID, Serial);
    

    // OLED: SDA=GPIO4(D2), SCL=GPIO5(D1)
    Wire.begin(12, 14);
    
    display.init();
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    display.clear();

    display.drawString(0, 0, "Fronius Primo");
    display.drawString(0, 12, "Modbus RTU");
    display.drawString(0, 26, "ID Modbus: " + String(MODBUS_ID));
    display.drawString(0, 38, "Baud: " + String(MODBUS_BAUD));
    display.drawString(0, 52, "Iniciando...");
    display.display();

    DBGLN("\n=== Fronius Primo RTU Reader ===");
    DBGF("Modbus ID  : %d\n", MODBUS_ID);
    DBGF("Baudrate   : %lu bps\n", MODBUS_BAUD);
    DBGLN("MAX3485    : auto-direction (sin DE/RE externo)");
    DBGLN("================================\n");

    delay(1500);
}

// =============================================================
void loop() {
    static uint32_t lastPoll = 0;
    if (millis() - lastPoll < POLL_INTERVAL_MS) return;
    lastPoll = millis();

    runReadCycle();
    updateOLED();
    printDebug();
}

// =============================================================
//  Ciclo completo: SF -> MPPT1 -> MPPT2 -> Bloque AC
// =============================================================
void runReadCycle() {
    inv.valid = false;

    // Scale factors (si fallan, continua con valores por defecto)
    if (!readMPPTScaleFactors()) {
        DBGLN("[WARN] SF no leidos, usando sf=-2 (x0.01)");
    }

    // String 1 — obligatorio
    if (!readMPPTString(1, inv.Idc1, inv.Vdc1, inv.Pdc1)) {
        DBGLN("[ERR] Fallo MPPT1 — abortando ciclo");
        return;
    }

    // String 2 — no fatal (puede haber un solo string activo)
    if (!readMPPTString(2, inv.Idc2, inv.Vdc2, inv.Pdc2)) {
        DBGLN("[WARN] Fallo MPPT2 — asumiendo 0");
        inv.Idc2 = inv.Vdc2 = inv.Pdc2 = 0.0f;
    }

    inv.PdcTotal = inv.Pdc1 + inv.Pdc2;

    // Bloque AC — obligatorio
    if (!readACBlock()) {
        DBGLN("[ERR] Fallo bloque AC — abortando ciclo");
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
            DBGF("[SF] DCA=%d DCV=%d DCW=%d\n", sf_DCA, sf_DCV, sf_DCW);
            return true;
        }
        DBGF("[SF] retry %d err=0x%02X\n", r + 1, res);
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

            DBGF("[MPPT%d] I=%.3fA V=%.2fV P=%.1fW\n", str, I, V, P);
            return true;
        }
        DBGF("[MPPT%d] retry %d err=0x%02X\n", str, r + 1, res);
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

            DBGF("[AC] I=%.3fA V=%.2fV P=%.1fW St=%s\n",
                 inv.Iac, inv.Vac, inv.Pac, stateStr(inv.state));
            return true;
        }
        DBGF("[AC] retry %d err=0x%02X\n", r + 1, res);
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
//  OLED — 128x64, text size 1 (6x8 px), 8 filas disponibles
// =============================================================
void updateOLED() {
    display.clear();
    display.setFont(ArialMT_Plain_10);

    if (!inv.valid) {
        display.drawString(0, 0, "FRONIUS PRIMO");
        display.drawLine(0, 11, 127, 11);
        display.drawString(0, 14, "Sin respuesta");
        display.drawString(0, 28, "Modbus ID: " + String(MODBUS_ID));
        display.drawString(0, 42, "Comprobar:");
        display.drawString(0, 54, "GND / ID / cables");
        display.display();
        return;
    }

    // Cabecera
    display.drawString(0, 0, "FRONIUS  " + String(stateStr(inv.state)));
    display.drawLine(0, 11, 127, 11);

    // DC Strings
    display.drawString(0, 12, "S1 " + String(inv.Idc1, 1) + "A " + String(inv.Vdc1, 0) + "V");
    display.drawString(0, 22, "S2 " + String(inv.Idc2, 1) + "A " + String(inv.Vdc2, 0) + "V");
    display.drawString(0, 32, "Pdc " + String(inv.PdcTotal, 0) + " W");

    display.drawLine(0, 43, 127, 43);

    // AC Data
    display.drawString(0, 45, "Pac " + String(inv.Pac, 0) + " W");
    display.drawString(0, 54, "V " + String(inv.Vac, 1) + "V  I " + String(inv.Iac, 2) + "A");

    display.display();
}

// =============================================================
//  Resultados por Serial1 (debug)
// =============================================================
void printDebug() {
    DBGLN("========================================");
    if (!inv.valid) {
        DBGLN("[ERROR] Ciclo de lectura fallido");
        DBGLN("========================================\n");
        return;
    }
    DBGF("[Estado]   %d - %s\n", inv.state, stateStr(inv.state));
    DBGLN("--- DC String 1 ---");
    DBGF("  I : %.3f A\n", inv.Idc1);
    DBGF("  V : %.2f V\n", inv.Vdc1);
    DBGF("  P : %.1f W\n", inv.Pdc1);
    DBGLN("--- DC String 2 ---");
    DBGF("  I : %.3f A\n", inv.Idc2);
    DBGF("  V : %.2f V\n", inv.Vdc2);
    DBGF("  P : %.1f W\n", inv.Pdc2);
    DBGF("  Pdc Total: %.1f W\n", inv.PdcTotal);
    DBGLN("--- AC ---");
    DBGF("  I : %.3f A\n", inv.Iac);
    DBGF("  V : %.2f V\n", inv.Vac);
    DBGF("  P : %.1f W\n", inv.Pac);
    DBGLN("========================================\n");
}