#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

SSD1306Wire display(0x3C, 12, 14);

// Credenciales
const char* ssid     = "MOVISTAR_1D80";
const char* password = "nhM9ing7k4793YnX74ni";

void displaySetUp();
void wifiSetUp();

void setup() {
  Serial.begin(74880);
  displaySetUp();
  wifiSetUp();
}

void loop() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);

  if(WiFi.status() == WL_CONNECTED){
    display.drawString(64, 10, "WiFi: CONECTADO");
    delay(2000);
    display.clear();
    display.drawString(64, 20, WiFi.hostname()); //Nombre del ESP8266
    display.drawString(64, 40, WiFi.localIP().toString()); //IP del ESP8266
  } else {
    display.drawString(64, 20, "Conectando...");
    // Intentar reconectar si se pierde
    WiFi.begin(ssid, password);
  }
  
  display.display();
  delay(5000); // Esperamos 5 segundos para no saturar el chip
}

void displaySetUp() {
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.flipScreenVertically();
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