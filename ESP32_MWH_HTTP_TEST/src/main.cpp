#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

// --- Credenciales ---
const char* ssid = "MOVISTAR_1D80";
const char* password = "nhM9ing7k4793YnX74ni";
const char* esiosToken = "76f5317763cf71beec91ede16a667d8428e1ff3b793d45665a3c803e5ea5e68e";

// -- Configuración NTP y Zona Horaria (Madrid) ---
const char* ntpServer = "pool.ntp.org";
const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

// URL Real del Indicador 1001 (Tarifa 2.0TD) filtrado para la Península
const char* urlEsios = "https://api.esios.ree.es/indicators/1001?start_date=2026-05-16T00:00:00Z&end_date=2026-05-16T23:59:59Z&geo_ids[]=8741";

// --- Temporizador (10 minutos) ---
unsigned long lastTimeRequest = 0;
const unsigned long timerDelay = 600000; 
float precioActualKWh = 0.0;

void obtenerPrecioESIOS() {
    if (WiFi.status() != WL_CONNECTED) return;

    //Obtención hora y fecha actual
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("[TIME] Error: No se pudo obtener la hora local");
        return;
    }

    //Formateo de la fecha actual
    char fechaHoy[11];
    strftime(fechaHoy, sizeof(fechaHoy), "%Y-%m-%d", &timeinfo);

    //Construimos URL dinámica para el día actual
    String urlDinamica = "https://api.esios.ree.es/indicators/1001?start_date=";
    urlDinamica += String(fechaHoy) + "T00:00:00Z&end_date=";
    urlDinamica += String(fechaHoy) + "T23:59:59Z&geo_ids[]=8741";

    Serial.print("\n[ESIOS] Consultando URL: ");
    Serial.println(urlDinamica);

    WiFiClientSecure *client = new WiFiClientSecure;
    client->setInsecure(); // Omitimos certificados pesados para proteger la RAM

    HTTPClient http;
    http.begin(*client, urlEsios);
    
    // --- Cabeceras Oficiales Verificadas ---
    http.addHeader("x-api-key", esiosToken); 
    http.addHeader("Accept", "application/json; application/vnd.esios-api.v1+json");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("User-Agent", "ESP32-DevKitV1-Client");

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
        // Filtramos el flujo en tiempo real: extraemos solo el valor numérico
        JsonDocument filter;
        filter["indicator"]["values"][0]["value"] = true;
        filter["indicator"]["values"][0]["datetime"] = true;

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));

        if (!error) {
            JsonArray values = doc["indicator"]["values"];

            char patronHora[10];
            snprintf(patronHora, sizeof(patronHora), "T%02d:00:00", timeinfo.tm_hour);
            
            bool encontrado = false;

            for (JsonObject v : values) {
                const char* datetimeStr = v["datetime"];
                
                if (datetimeStr != NULL && strstr(datetimeStr, patronHora) != NULL) {
                    float precioMWh = v["value"];
                    precioActualKWh = precioMWh / 1000.0;
                    encontrado = true;
                    break; // Salimos del bucle al encontrar la hora exacta
                }
            }
            
            if (encontrado) {
                Serial.println("====================================");
                Serial.printf(" HORA LOCAL: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                Serial.printf(" PRECIO FILTRADO PVPC: %.4f €/kWh\n", precioActualKWh);
                Serial.println("====================================");
            } else {
                Serial.printf("[ESIOS] Tramo '%s' no disponible en el JSON todavía.\n", patronHora);
            }

        } else {
            Serial.printf("[JSON] Error de parseo: %s\n", error.c_str());
        }
    } else {
        Serial.printf("[HTTP] Error ESIOS: %d\n", httpResponseCode);
    }

    http.end();
    delete client; // Liberación estricta de memoria
}

void setup() {
    Serial.begin(115200);
    
    WiFi.begin(ssid, password);
    Serial.print("Conectando Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n[WIFI] ¡Conectado!");

    Serial.println("[NTP] Sincronizando hora...");
    configTzTime(TZ_INFO, ntpServer);

    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n[NTP] Reloj del sistema sincronizado");

    // Primera consulta al iniciar
    obtenerPrecioESIOS();
    lastTimeRequest = millis();
}

void loop() {
    unsigned long currentMillis = millis();

    // Cronómetro no bloqueante de 10 minutos
    if (currentMillis - lastTimeRequest >= timerDelay) {
        lastTimeRequest = currentMillis;
        obtenerPrecioESIOS();
    }
}