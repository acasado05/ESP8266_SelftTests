#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Sensores
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

struct MedidasAmbientales {
  float tempAHT;
  float humAHT;
  float tempBMP;
  float tempAmbFinal;
};

// Variables de tiempo y control
unsigned long anteriorMillisLog = 0; // Para el control del minuto
const long intervaloLog = 20000;      // 20.000 ms = 20 segundos
int estadoLed = HIGH;
unsigned long anteriorMillisBlink = 0;
const int MUESTRAS_PROMEDIO = 20;

// Credenciales WiFi
const char* ssid     = "TP-LINK_C062";
const char* password = "77817570";
const char* MY_TZ = "CET-1CEST,M3.5.0,M10.5.0/3"; // Zona horaria de Madrid

// Prototipos
void wifiSetUp();
void blinkLed(int parpadeo);
void logDatosSerial();
String getTimeStamp();
void setNTP();
void pruebaConexion();
MedidasAmbientales realizarMedida (void);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Espera a que el puerto serie esté listo
  
  Serial.println("\n[SISTEMA] Puerto serie detectado. Iniciando configuración...");
  delay(2000);

  wifiSetUp();

  /* Espera activa */
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Inicialización de sensores
  if(!aht.begin()) {
    Serial.println("Error: No se encontró AHT20");
  }
  if(!bmp.begin(BMP280_ADDRESS)) {
    Serial.println("Error: No se encontró BMP280");
  }
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, estadoLed);

  // Configuración de hora NTP
  setNTP();  // Configura el NTP para sincronizar la hora
  pinMode(LED_BUILTIN, OUTPUT);
  pruebaConexion();
  Serial.println("\nSistema listo. Esperando sincronización horaria...");
}

void loop() {
  
  blinkLed(1000); // El LED parpadea lento si hay WiFi
  unsigned long currentMillis = millis();

  if (currentMillis - anteriorMillisLog >= intervaloLog) {
      anteriorMillisLog = currentMillis;

      if (WiFi.status() == WL_CONNECTED) {
          // Lógica principal: Ejecutar cada 1 minuto   
          logDatosSerial(); // Función que imprime datos por el puerto serie 
      } else {
        blinkLed(100); // Parpadeo rápido si se pierde el WiFi
      }
    }
}

void wifiSetUp() {
  Serial.print("\nConectando a: ");
  Serial.println(ssid);
  WiFi.hostname("ESP32_t0rt1s");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void logDatosSerial() {
  MedidasAmbientales datos = realizarMedida();

  Serial.println(F("========================================"));
  Serial.printf("Timestamp: %s\n", getTimeStamp().c_str());
  Serial.println(F("----------------------------------------"));

  Serial.printf("%-16s : %6.2f ºC\n", "Temp. AHT20",  datos.tempAHT);
  Serial.printf("%-16s : %6.2f ºC\n", "Temp. BMP280", datos.tempBMP);
  Serial.printf("%-16s : %6.2f ºC\n", "Temp. Media (T)", datos.tempAmbFinal);
  Serial.printf("%-16s : %6.2f %%\n",  "Humedad (RH)",   datos.humAHT);
  
  Serial.println(F("========================================\n"));
}

void blinkLed(int parpadeo) {
  unsigned long actualMillis = millis();
  if (actualMillis - anteriorMillisBlink >= (unsigned long)parpadeo) {
    anteriorMillisBlink = actualMillis;
    estadoLed = !estadoLed;
    digitalWrite(LED_BUILTIN, estadoLed);
  }
}

void setNTP() {
  configTzTime(MY_TZ, "pool.ntp.org", "time.google.com");
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

MedidasAmbientales realizarMedida (void){

  sensors_event_t h, t;
  float sumaTempAHT = 0.0f; float sumaHumAHT  = 0.0f; float sumaTempBMP = 0.0f;

  MedidasAmbientales medidas;

  for(int i = 0; i < MUESTRAS_PROMEDIO; i++){
    aht.getEvent(&h, &t);
    sumaTempAHT += t.temperature;
    sumaHumAHT += h.relative_humidity;
    sumaTempBMP += bmp.readTemperature();
    delay(50);
  }

  medidas.tempAHT = sumaTempAHT / MUESTRAS_PROMEDIO;
  medidas.humAHT = sumaHumAHT / MUESTRAS_PROMEDIO;
  medidas.tempBMP = sumaTempBMP / MUESTRAS_PROMEDIO;
  medidas.tempAmbFinal = (medidas.tempAHT + medidas.tempBMP) / 2.0;
  
  return medidas;
}