#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

// --- Credenciales WiFi ---
const char* ssid = "T0rt1s_A54";
const char* password = "tortis007";
const char* esiosToken = "TU_TOKEN_AQUÍ";

// --- DNS de Google (Crucial para evitar el error DNS Failed) ---
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// --- Variables de Control ---
unsigned long lastTimeRequest = 0;
const unsigned long timerDelay = 600000; // 10 minutos
float precioActualKWh = 0.0;

void obtenerPrecioOficial() {
    if (WiFi.status() != WL_CONNECTED) return;

    WiFiClientSecure *client = new WiFiClientSecure;
    client->setInsecure();

    HTTPClient http;
    
    // Consultamos el indicador 1013 (PVPC)
    // Usamos api.esios.ree.es (Dominio oficial de Red Eléctrica)
    http.begin(*client, "https://api.esios.ree.es/indicators/1013");
    
    // Headers obligatorios para ESIOS
    http.addHeader("Accept", "application/json; application/vnd.esios-api.v1+json");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-token-auth", esiosToken);

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
        String payload = http.getString();
        JsonDocument doc;
        deserializeJson(doc, payload);

        // ESIOS devuelve muchos datos, cogemos el último valor (el más actual)
        JsonArray values = doc["indicator"]["values"];
        float precioMWh = values[values.size() - 1]["value"]; 
        float precioKWh = precioMWh / 1000.0;

        Serial.printf("\n[ESIOS] Precio oficial Red Eléctrica: %.4f €/kWh\n", precioKWh);
    } else {
        Serial.printf("[ESIOS] Error: %d\n", httpResponseCode);
    }
    
    http.end();
    delete client;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    WiFi.begin(ssid, password);
    Serial.print("[WIFI] Conectando");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\n[WIFI] ¡Conectado!");
    Serial.print("[WIFI] IP asignada por el router: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WIFI] Puerta de enlace (Gateway): ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[WIFI] DNS Principal: ");
    Serial.println(WiFi.dnsIP()); // Vamos a ver qué DNS te da el router por defecto

    // Prueba de fuego: ¿Podemos resolver una IP de Google?
    IPAddress apiIP;
    if (WiFi.hostByName("api.preciodelaluz.org", apiIP)) {
      Serial.print("[DNS] ¡API Localizada! IP: ");
      Serial.println(apiIP);
    } else {
      Serial.println("[DNS] La API sigue sin responder al nombre...");
    }

    delay(2000);
    obtenerPrecioLuz();
}

void loop() {
    unsigned long currentMillis = millis();

    // Temporizador no bloqueante
    if (currentMillis - lastTimeRequest >= timerDelay) {
        lastTimeRequest = currentMillis;
        obtenerPrecioLuz();
    }

    // El resto de tu hardware (Inversor, ADS1115) iría aquí abajo
}