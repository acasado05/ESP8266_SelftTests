#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <SPI.h>
#include <SD.h>

// ─── Configuración Wi-Fi ─────────────────────────────────────────────
const char* ssid       = "TP-LINK_C062";
const char* password   = "77817570";

// ─── Configuración NTP (Servidor de Hora) ────────────────────────────
const char* ntpServer  = "pool.ntp.org";
// Cadena POSIX para la hora peninsular (Ajusta invierno/verano automáticamente)
const char* tzInfo     = "CET-1CEST,M3.5.0,M10.5.0/3"; 

// ─── Configuración MicroSD ───────────────────────────────────────────
#define SD_SCK  12
#define SD_MISO 13
#define SD_MOSI 11
#define SD_CS   10

SPIClass spiSD(FSPI);
const char* filename = "/log_temporal.csv";

// ─── Cronómetro no bloqueante ────────────────────────────────────────
unsigned long tiempoAnterior = 0;
const unsigned long intervaloEscritura = 10000; // 10 segundos en milisegundos

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== Iniciando Sistema Datalogger NTP ===");

    // 1. Conexión Wi-Fi
    Serial.printf("Conectando a %s ", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n[OK] Wi-Fi conectado.");

    // 2. Sincronización de Hora (NTP)
    Serial.println("Sincronizando hora con servidor NTP...");
    configTzTime(tzInfo, ntpServer);
    
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("[ERROR] No se pudo obtener la hora. Revisa la conexión.");
        return;
    }
    Serial.println("[OK] Hora sincronizada correctamente.");

    // 3. Inicialización de la MicroSD
    spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, spiSD, 4000000)) {
        Serial.println("[ERROR CRÍTICO] Fallo al montar la tarjeta SD.");
        return;
    }
    Serial.println("[OK] Tarjeta MicroSD lista.");

    // Escribir cabeceras si el archivo no existe
    if (!SD.exists(filename)) {
        File file = SD.open(filename, FILE_APPEND);
        if (file) {
            file.println("Fecha_Hora,Prueba_Valor");
            file.close();
            Serial.println("  -> Archivo nuevo creado con cabeceras.");
        }
    }
    
    Serial.println("=== Sistema funcionando. Guardando cada 10s ===");
}

void loop() {
    // Comprobamos si han pasado 10 segundos (sin usar delay!)
    if (millis() - tiempoAnterior >= intervaloEscritura) {
        tiempoAnterior = millis(); // Reseteamos el cronómetro

        struct tm timeinfo;
        if (!getLocalTime(&timeinfo)) {
            Serial.println("Error al obtener la hora local.");
            return;
        }

        // Formateamos la hora como un String limpio: "YYYY-MM-DD HH:MM:SS"
        char timeStringBuff[50];
        strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);

        // Simulamos un valor aleatorio o contador para guardar junto a la hora
        int valorAleatorio = random(100, 999);
        String logLinea = String(timeStringBuff) + "," + String(valorAleatorio);

        // Abrimos el archivo, escribimos y cerramos INMEDIATAMENTE
        File file = SD.open(filename, FILE_APPEND);
        if (file) {
            file.println(logLinea);
            file.close(); // Crucial cerrar para asegurar los datos contra cortes de luz
            Serial.println("[SD] Guardado: " + logLinea);
        } else {
            Serial.println("[ERROR] No se pudo abrir el archivo para escribir.");
        }
    }
    
    // Aquí el microcontrolador está libre para hacer miles de cosas más
    // mientras espera a que pasen los 10 segundos.
}