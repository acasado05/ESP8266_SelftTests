#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <time.h>

//Variables
SSD1306Wire display(0x3C, 12, 14);
unsigned long anteriorMillis = 0;
unsigned long segundo = 0;
int estadoLed = HIGH;
unsigned long mensajeMostrado = 0;
unsigned long tiempoInicioEstado = 0;
const char* temp = "25.3ºC";
const char* hum = "60%";
unsigned long tiempoIntercalado = 0;
int estadoPantalla = 0;

const char* MY_TZ = "CET-1CEST,M3.5.0,M10.5.0/3"; // Zona horaria de Madrid (CET/CEST)

// Credenciales
/*const char* ssid     = "MOVISTAR_1D80";
const char* password = "nhM9ing7k4793YnX74ni"; */
const char* ssid     = "T0rt1s_A54";
const char* password = "tortis007";

// Prototipos de funciones
void displaySetUp();
void wifiSetUp();
void blinkLed (int parpadeo);
void mostrarHora (void);
void mostrarTempHum (void);

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

  if(WiFi.status() == WL_CONNECTED) {
    blinkLed(1000);
    // Si acaba de conectar, mostramos el mensaje una vez
    switch(mensajeMostrado) {
      case 0:
        display.clear();
        display.drawString(64, 30, "WiFi: CONECTADO");
        display.display();
        delay(2000); // Mostramos el mensaje durante 2 segundos
        tiempoInicioEstado = millis(); // Guardamos el tiempo de inicio del case 1
        mensajeMostrado = 1;

        break;
      
      case 1:
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.setFont(ArialMT_Plain_10);
        display.drawString(64, 0, "--- DATOS DE RED ---");
        display.drawString(64, 20, WiFi.hostname());
        display.drawString(64, 40, WiFi.localIP().toString());
        display.display();

        /* El mensaje solo se muestra durante 3 segundos*/
        if(millis () - tiempoInicioEstado >= 3000) {
          mensajeMostrado = 2;
        }
        break;
      
      //Muestra la hora actualizándose cada segundo
      case 2:
      if (millis() - tiempoIntercalado >= 10000){
          tiempoIntercalado = millis(); 
          estadoPantalla = (estadoPantalla == 0) ? 1 : 0; // Alterna entre mostrar hora y temp/hum
        }
        if (millis() - segundo >= 1000){
          segundo = millis(); 
          if (estadoPantalla == 0) {
            mostrarHora();
          } else {
            mostrarTempHum();
          }
        }
        break;
      
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
    //Serial.printf("%02d/%02d/%04d %02d:%02d:%02d\n", p_tm->tm_mday, p_tm->tm_mon + 1, p_tm->tm_year + 1900, p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    
    //HORA EN GRANDE
    display.setFont(ArialMT_Plain_24);
    char horaBuffer[10];
    sprintf(horaBuffer, "%02d:%02d:%02d", p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
    display.drawString(64, 20, horaBuffer);
    
    //FECHA EN PEQUEÑO
    display.setFont(ArialMT_Plain_10);
    char fechaBuffer[40];
    sprintf(fechaBuffer, "%02d/%02d/%04d", p_tm->tm_mday, p_tm->tm_mon + 1, p_tm->tm_year + 1900);
    display.drawString(64, 50, fechaBuffer);

    display.display();
  }else{
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    display.drawString(64, 30, "Sin sincronización");
    display.display();
  }
}

void mostrarTempHum (void){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_24);
  display.drawString(64, 10, temp);
  display.drawString(64, 30, hum);
  display.display();
}