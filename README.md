# 🌡️ Estación Meteorológica con ESP8266 y PlatformIO

¡Un proyecto para monitorizar la temperatura y humedad en tiempo real con un ESP8266, sensores y una pantalla OLED!

## 📜 Descripción

Este repositorio contiene el código para una estación meteorológica compacta y personalizable. El proyecto está desarrollado con [PlatformIO](https://platformio.org/) y utiliza un microcontrolador ESP8266 para leer datos de sensores de temperatura y humedad, sincronizar la hora a través de internet y mostrar la información de forma clara y concisa.

## ✨ Características

*   **Monitorización en tiempo real**: Lectura de temperatura y humedad de múltiples sensores.
*   **Doble sensor de temperatura**: Utiliza un AHT20 y un BME280 para obtener mediciones más precisas.
*   **Pantalla OLED**: Muestra la información de forma clara y legible en una pantalla SSD1306.
*   **Conectividad WiFi**: Se conecta a tu red local para sincronizar la hora y futuras ampliaciones (¡como un servidor web!).
*   **Registro de datos**: Imprime los datos de los sensores en el puerto serie en un formato fácil de procesar.
*   **Sincronización NTP**: Obtiene la fecha y hora exactas de servidores NTP.
*   **Indicador LED**: Parpadea para mostrar el estado de la conexión WiFi.
*   **Autodiagnóstico**: Incluye una prueba de la memoria flash al inicio.

## 🛠️ Hardware

*   **Microcontrolador**: ESP8266 (NodeMCU v2)
*   **Sensores**:
    *   Adafruit AHT20 (temperatura y humedad)
    *   Adafruit BME280 (temperatura)
*   **Pantalla**: SSD1306 OLED (I2C)

## 🔧 Software y Librerías

Este proyecto está construido sobre el framework de Arduino y utiliza las siguientes librerías:

*   `thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays`
*   `adafruit/Adafruit Unified Sensor`
*   `adafruit/Adafruit BME280 Library`
*   `adafruit/Adafruit AHTX0`

## ⚙️ Configuración

1.  **WiFi**: Modifica las siguientes líneas en `src/main.cpp` con las credenciales de tu red WiFi:

    ```cpp
    const char* ssid     = "TU_SSID";
    const char* password = "TU_PASSWORD";
    ```

2.  **Zona horaria**: Ajusta la variable `MY_TZ` en `src/main.cpp` para que coincida con tu zona horaria. El formato es POSIX.

    ```cpp
    const char* MY_TZ = "CET-1CEST,M3.5.0,M10.5.0/3"; // Ejemplo para Madrid, España
    ```

## 🚀 Uso

1.  Clona este repositorio.
2.  Abre el proyecto con [Visual Studio Code](https://code.visualstudio.com/) y la extensión de [PlatformIO](https://platformio.org/platformio-ide).
3.  Modifica la configuración como se indica en la sección anterior.
4.  Conecta tu placa ESP8266.
5.  Compila y sube el código a la placa.
6.  Abre el monitor serie a `115200` baudios para ver los registros y la información de la memoria flash.

## 🤝 Contribuciones

Las contribuciones son bienvenidas. Si tienes ideas para mejorar el proyecto, por favor, abre un *issue* o envía un *pull request*.

## 📄 Licencia

Este proyecto está sin licencia. Siéntete libre de usarlo como quieras.
