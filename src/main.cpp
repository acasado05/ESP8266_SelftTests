#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <time.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Sensores
SSD1306Wire display(0x3C, 12, 14);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Variables de tiempo y control
unsigned long anteriorMillisLog = 0; // Para el control del minuto
const long intervaloLog = 60000;      // 60.000 ms = 1 minuto
int estadoLed = HIGH;
unsigned long anteriorMillisBlink = 0;

const char* MY_TZ = "CET-1CEST,M3.5.0,M10.5.0/3"; // Zona horaria de Madrid

// Credenciales WiFi
const char* ssid     = "TP-LINK_C062";
const char* password = "77817570";
/*const char* ssid     = "T0rt1s_A54";
const char* password = "tortis007"; */

// Prototipos
void wifiSetUp();
void blinkLed(int parpadeo);
void logDatosSerial();
void displaySetUp();

void setup() {
  Serial.begin(74880);
  while (!Serial) { delay(10); } // Espera a que el puerto serie esté listo
  Serial.println("\n[SISTEMA] Puerto serie detectado. Iniciando configuración...");
  delay(2000);
  displaySetUp();

  // Inicialización de sensores
  if(!aht.begin()) {
    Serial.println("Error: No se encontró AHT10/AHT20");
  }
  if(!bmp.begin(BMP280_ADDRESS)) {
    Serial.println("Error: No se encontró BMP280");
  }

  wifiSetUp();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, estadoLed);

  // Configuración de hora NTP
  configTime(MY_TZ, "pool.ntp.org", "time.google.com");
  Serial.println("\nSistema listo. Esperando sincronización horaria...");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    blinkLed(1000); // El LED parpadea lento si hay WiFi

    unsigned long currentMillis = millis();

    // Lógica principal: Ejecutar cada 1 minuto
    if (currentMillis - anteriorMillisLog >= intervaloLog) {
      anteriorMillisLog = currentMillis;
      logDatosSerial(); // Función que imprime datos por el puerto serie
    }
  } else {
    blinkLed(100); // Parpadeo rápido si se pierde el WiFi
  }
}

void wifiSetUp() {
  Serial.print("Conectando a: ");
  Serial.println(ssid);
  WiFi.hostname("ESP8266_t0rt1s");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void logDatosSerial() {
  // 1. Obtener Fecha y Hora
  time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);

  // 2. Leer Sensores
  sensors_event_t humidity, temp_aht;
  aht.getEvent(&humidity, &temp_aht);
  float temp_bmp = bmp.readTemperature();
  float tempPromedio = (temp_aht.temperature + temp_bmp) / 2.0;

  // 3. Imprimir por Serie (Formato CSV o Tabular)
  if (p_tm->tm_year > 70) { // Solo imprime si la hora está sincronizada
    Serial.print("[LOG] ");
    // Fecha y Hora
    Serial.printf("%02d/%02d/%04d %02d:%02d:%02d | ", 
                  p_tm->tm_mday, p_tm->tm_mon + 1, p_tm->tm_year + 1900, 
                  p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
    
    // Sensores
    Serial.printf("Temp: %.2f ºC | Hum: %.2f %% | BMP280: %.2f ºC | AHT20: %.2f ºC\n", 
                  tempPromedio, humidity.relative_humidity, temp_bmp, temp_aht.temperature);
  } else {
    Serial.println("[!] Esperando sincronización NTP para registrar datos...");
  }
}

void blinkLed(int parpadeo) {
  unsigned long actualMillis = millis();
  if (actualMillis - anteriorMillisBlink >= (unsigned long)parpadeo) {
    anteriorMillisBlink = actualMillis;
    estadoLed = !estadoLed;
    digitalWrite(LED_BUILTIN, estadoLed);
  }
}

void displaySetUp() {
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.clear();
}