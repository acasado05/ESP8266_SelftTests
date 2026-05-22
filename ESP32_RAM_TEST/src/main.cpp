#include <Arduino.h>
#include "esp_heap_caps.h"

void setup() {
  // Inicializamos el puerto serie (recuerda mantener el cable en el puerto UART)
  Serial.begin(115200);
  
  // Pequeño retardo para dar tiempo a abrir el monitor serie de PlatformIO
  delay(2000); 

  Serial.println("\n=== DIAGNÓSTICO DE MEMORIA ESP32-S3 ===");

  // --- 1. Análisis de la SRAM (Memoria volátil para ejecución) ---
  // getHeapSize() devuelve la RAM disponible para variables dinámicas,
  // restando ya lo que ocupa el sistema base (FreeRTOS, WiFi, etc.)
  uint32_t ram_total = ESP.getHeapSize();
  uint32_t ram_libre = ESP.getFreeHeap();
  uint32_t total_sram_interna = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
  
  // getMaxAllocHeap() devuelve el bloque de memoria contiguo más grande.
  // ¡Este es el número MÁS IMPORTANTE para TensorFlow Lite!
  uint32_t ram_max_bloque = ESP.getMaxAllocHeap(); 

  Serial.println("\n[SRAM Interna]");
  Serial.printf("- SRAM Interna Total (Gestor de memoria): %u bytes\n", heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
  Serial.printf("- Total disponible para programa:         %u bytes (%.2f KB)\n", ram_total, ram_total / 1024.0);
  Serial.printf("- Libre en este momento:                  %u bytes (%.2f KB)\n", ram_libre, ram_libre / 1024.0);
  Serial.printf("- Mayor bloque contiguo:                  %u bytes (%.2f KB) <-- Límite real para la IA\n", ram_max_bloque, ram_max_bloque / 1024.0);

  // --- 2. Análisis de la Flash (Almacenamiento de código y modelo) ---
  uint32_t flash_size = ESP.getFlashChipSize();
  uint32_t flash_speed = ESP.getFlashChipSpeed();

  Serial.println("\n[Memoria Flash]");
  Serial.printf("- Tamaño total:  %u bytes (%.2f MB)\n", flash_size, flash_size / (1024.0 * 1024.0));
  Serial.printf("- Velocidad:     %u MHz\n", flash_speed / 1000000);
  
  Serial.println("=======================================\n");
}

void loop() {
  // Solo necesitamos leerlo una vez al arrancar
  delay(10000);
}