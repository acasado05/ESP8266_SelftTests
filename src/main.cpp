#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

//Variables
SSD1306Wire display(0x3C, 12, 14);
unsigned long anteriorMillis = 0;
unsigned long ultimaActualizaciónInfo = 0;
int estadoLed = HIGH;
bool mensajeMostrado = false;

// Credenciales
const char* ssid     = "MOVISTAR_1D80";
const char* password = "nhM9ing7k4793YnX74ni";

// Prototipos de funciones
void displaySetUp();
void wifiSetUp();
void blinkLed (int parpadeo);

void setup() {
  Serial.begin(74880);
  delay(2000); // Esperamos 2 segundos a que se estabilice el Serial
  displaySetUp();
  wifiSetUp();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, estadoLed);
}

void loop() {
  // 1. El LED se actualiza CONSTANTEMENTE (sin bloqueos)
  if(WiFi.status() == WL_CONNECTED) {
    blinkLed(1000); 
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