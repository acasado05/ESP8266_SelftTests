#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <time.h> 

// --- Credenciales ---
const char* ssid = "T0rt1s_A54";
const char* password = "tortis007";
const char* esiosToken = "76f5317763cf71beec91ede16a667d8428e1ff3b793d45665a3c803e5ea5e68e";

// -- Configuración NTP y Zona Horaria (Madrid) ---
const char* ntpServer = "pool.ntp.org";
const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

// --- Temporizador (10 minutos) ---
unsigned long lastTimeRequest = 0;
const unsigned long timerDelay = 600000; 
float precioActualKWh = 0.0;

void obtenerPrecioESIOS() {
    if (WiFi.status() != WL_CONNECTED) return;

    // Obtención hora y fecha actual
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("[TIME] Error: No se pudo obtener la hora local");
        return;
    }

    // Formateo de la fecha actual
    char fechaHoy[11];
    strftime(fechaHoy, sizeof(fechaHoy), "%Y-%m-%d", &timeinfo);

    // Construimos URL dinámica para el día actual
    String urlDinamica = "https://api.esios.ree.es/indicators/1001?start_date=";
    urlDinamica += String(fechaHoy) + "T00:00:00Z&end_date=";
    urlDinamica += String(fechaHoy) + "T23:59:59Z&geo_ids[]=8741";

    Serial.print("\n[ESIOS] Consultando URL: ");
    Serial.println(urlDinamica);

    WiFiClientSecure *client = new WiFiClientSecure;
    client->setInsecure(); // Omitimos certificados pesados para proteger la RAM

    HTTPClient http;
    // --- CORRECCIÓN CRUCIAL ---
    // Usamos urlDinamica en lugar de la constante urlEsios estática
    http.begin(*client, urlDinamica); 
    
    // --- Cabeceras Oficiales Verificadas ---
    http.addHeader("x-api-key", esiosToken); 
    http.addHeader("Accept", "application/json; application/vnd.esios-api.v1+json");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("User-Agent", "ESP32-S3-Client");

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
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
                    break; 
                }
            }
            
            if (encontrado) {
                Serial.println("====================================");
                Serial.printf(" HORA LOCAL DEL S3: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
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
    delete client; 
}

void setup() {
    Serial.begin(115200);
    
    // Configuramos el LED interno (En las placas oficiales suele ser el pin 48 o LED_BUILTIN)
    #ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Encendemos el LED al arrancar
    #endif

    // Espera activa para el USB nativo
    while (!Serial) {
        delay(10); 
    }
    
    // En cuanto abras el monitor, pasará de aquí y verás esto instantáneamente:
    Serial.println("\n====================================");
    Serial.println("[S3] ¡CONEXIÓN ESTABLECIDA CON EL PC!");
    Serial.println("====================================");
    
    WiFi.begin(ssid, password);
    Serial.print("Conectando Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        // Hacemos parpadear el LED mientras busca WiFi
        #ifdef LED_BUILTIN
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        #endif
    }
    Serial.println("\n[WIFI] ¡Conectado!");
    
    #ifdef LED_BUILTIN
    digitalWrite(LED_BUILTIN, LOW); // Apagamos el LED al conectar con éxito
    #endif

    Serial.println("[NTP] Sincronizando hora...");
    configTzTime(TZ_INFO, ntpServer);

    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n[NTP] Reloj sincronizado.");

    obtenerPrecioESIOS();
    lastTimeRequest = millis();
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastTimeRequest >= timerDelay) {
        lastTimeRequest = currentMillis;
        obtenerPrecioESIOS();
    }
}