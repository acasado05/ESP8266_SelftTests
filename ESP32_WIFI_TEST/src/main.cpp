#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>

/* Credenciales de la red Wi-Fi */
const char* ssid = "TP-LINK_C062";
const char* password = "77817570";
const char* MY_TZ = "CET-1CEST,M3.5.0,M10.5.0/3"; // Zona horaria de Madrid

/* Variables LED AZUL*/
const int LED_AZUL = 2;
unsigned long anteriorMillisBlink = 0;
int estadoLed = LOW;

/* Variables Hora */
unsigned long anteriorMillisHora = 0;
const unsigned long intervaloHora = 10000; // 10 segundos

void wifiSetUp();
void pruebaConexion();
String getTimeStamp();
void setNTP();
void blinkLed(int parpadeo);


void setup() {
  Serial.begin(9600);
  delay(1000); // Pequeña pausa para asegurar que el monitor serial esté listo

  wifiSetUp();

  /* Espera activa */
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  delay(2000); // Espera un poco para que la conexión Wi-Fi se establezca

  setNTP();  // Configura el NTP para sincronizar la hora
  pinMode(LED_BUILTIN, OUTPUT);
  pruebaConexion(); // Llama a la función de prueba de conexión
}

void loop() {
  // put your main code here, to run repeatedly:
  blinkLed(1000);

  unsigned long actualMillis = millis();
  
  if (actualMillis - anteriorMillisHora >= intervaloHora) {
    anteriorMillisHora = actualMillis;
    
    if (WiFi.status() == WL_CONNECTED) {
      
      Serial.print(F("Timestamp -> "));
      Serial.println(getTimeStamp());
    }
  }
}

void pruebaConexion() {
  Serial.println(F("--- MODULAR TEST: INTERNET CONNECTION ---"));
  // Llama a tu función configurada
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[OK] IP asignada: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("[OK] Intensidad señal (RSSI): "));
    Serial.print(WiFi.RSSI());
    Serial.println(F(" dBm"));
  } else {
    Serial.println(F("[ERROR] No se pudo conectar al SSID: TP-LINK_C062"));
  }
}

void wifiSetUp() {
  Serial.print("\nConectando a: ");
  Serial.println(ssid);
  WiFi.hostname("ESP32_t0rt1s");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

String getTimeStamp(){
  time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);
  char timestamp[30];

  if(p_tm->tm_year > 70){ // Verifica que el año sea posterior a 1970
    sprintf(timestamp, "%02d/%02d/%04d %02d:%02d:%02d", 
            p_tm->tm_mday, 
            p_tm->tm_mon + 1, 
            p_tm->tm_year + 1900,
            p_tm->tm_hour, 
            p_tm->tm_min, 
            p_tm->tm_sec);

    return String(timestamp);
  } else {
    return String("Error conexión NTP");
  }
}

void setNTP() {
  configTzTime(MY_TZ, "pool.ntp.org", "time.google.com");
}

void blinkLed(int parpadeo) {
  unsigned long actualMillis = millis();
  if (actualMillis - anteriorMillisBlink >= (unsigned long)parpadeo) {
    anteriorMillisBlink = actualMillis;
    estadoLed = !estadoLed;
    digitalWrite(LED_AZUL, estadoLed);
  }
}
