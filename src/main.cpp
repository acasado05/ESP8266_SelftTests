#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <time.h>

//Variables
SSD1306Wire display(0x3C, 12, 14);
unsigned long anteriorMillis = 0;
unsigned long ultimaActualizaciónInfo = 0;
unsigned long segundo = 0;
int estadoLed = HIGH;
bool mensajeMostrado = false;

const char* MY_TZ = "CET-1CEST,M3.5.0,M10.5.0/3"; // Zona horaria de Madrid (CET/CEST)

// Credenciales
const char* ssid     = "MOVISTAR_1D80";
const char* password = "nhM9ing7k4793YnX74ni";

// Prototipos de funciones
void displaySetUp();
void wifiSetUp();
void blinkLed (int parpadeo);
void mostrarHora (void);

void setup() {
  Serial.begin(74880);
  delay(2000); // Esperamos 2 segundos a que se estabilice el Serial
  displaySetUp();
  wifiSetUp();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, estadoLed);
  configTime(MY_TZ, "pool.ntp.org", "time.google.com");
  Serial.println("\nEsperando a sincronización horaria...");
}

void loop() {
  // 1. El LED se actualiza CONSTANTEMENTE (sin bloqueos)
  if(WiFi.status() == WL_CONNECTED) {
    blinkLed(1000);

    if (millis() - segundo >= 1000){
      segundo = millis(); 
      mostrarHora(); 
    }

  } else {
    blinkLed(100);
    mensajeMostrado = false;
    
    display.clear();
    display.drawString(64, 20, "Conectando...");
    display.display();
    // Nota: WiFi.begin no suele ser necesario en el loop, el ESP reconecta solo
  }

  // 2. Lógica de la pantalla cuando hay WiFi
  if(WiFi.status() == WL_CONNECTED) {
    
    // Si acaba de conectar, mostramos el mensaje una vez
    if(!mensajeMostrado) {
      display.clear();
      display.drawString(64, 30, "WiFi: CONECTADO");
      display.display();
      
      // Usamos una pequeña espera aquí solo porque es una vez
      delay(2000); 
      mensajeMostrado = true;
      display.clear();
    }
    
    // 3. ACTUALIZACIÓN DE INFO CADA 5 SEGUNDOS (Sin bloquear el LED)
    if (millis() - ultimaActualizaciónInfo >= 5000) {
      ultimaActualizaciónInfo = millis();
      
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.setFont(ArialMT_Plain_10);
      display.drawString(64, 0, "--- DATOS DE RED ---");
      display.drawString(64, 20, WiFi.hostname());
      display.drawString(64, 40, WiFi.localIP().toString());
      display.display();
    }
  }
}

void displaySetUp() {
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(64, 20, "Iniciando...");
  display.display();
}

void wifiSetUp() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.hostname("ESP8266_t0rt1S"); //Nombre del ESP8266

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // Feedback visual en pantalla mientras conecta
    display.clear();
    display.drawString(64, 10, "Conectando...");
    counter++;

    if (counter > 40) { // Si tarda más de 20 seg, reiniciar WiFi
      WiFi.begin(ssid, password);
      counter = 0;
    }
  }

  Serial.println("\nConectado!");
}

void blinkLed (int parpadeo){
  unsigned long actualMillis = millis();

  if(actualMillis - anteriorMillis >= (unsigned long)parpadeo){
    anteriorMillis = actualMillis;
    estadoLed = (estadoLed == LOW) ? HIGH : LOW;
    digitalWrite(LED_BUILTIN, estadoLed);

  }
}

void mostrarHora (void){
  time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);

  if(p_tm->tm_year > 70){ //Si y solo si el año es mayor a 1970.
    Serial.printf("%02d/%02d/%04d %02d:%02d:%02d\n", 
                  p_tm->tm_mday, 
                  p_tm->tm_mon + 1, 
                  p_tm->tm_year + 1900, 
                  p_tm->tm_hour, 
                  p_tm->tm_min, 
                  p_tm->tm_sec);
  }
}