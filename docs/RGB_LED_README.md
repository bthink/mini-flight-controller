# RGB LED Module - ESP32-C6 Flight Controller

## Przegląd
Moduł RGB LED zapewnia zaawansowane sterowanie adresowalnym LED RGB (WS2812) na ESP32-C6 Flight Controller. Obsługuje różne tryby animacji i wzorce statusu dla aplikacji flight controllera.

## Konfiguracja Sprzętowa
- **Pin**: GPIO 8 (PIN_RGB_LED w config.h)
- **Typ LED**: WS2812 adresowalny LED strip
- **Protokół**: Single-wire communication via RMT peripheral
- **Zasilanie**: 3.3V lub 5V (w zależności od LED)

## Funkcjonalności

### Tryby Animacji
1. **RGB_MODE_STATIC** - Statyczny kolor
2. **RGB_MODE_BLINK** - Mruganie z cyklicznymi kolorami
3. **RGB_MODE_RAINBOW** - Płynne przejścia kolorów tęczy
4. **RGB_MODE_BREATHE** - Efekt oddychania z konfigurowalnymi kolorami
5. **RGB_MODE_FLIGHT_STATUS** - Wzorce statusu flight controllera

### Predefiniowane Kolory
```c
RGB_RED     {255, 0, 0}
RGB_GREEN   {0, 255, 0}
RGB_BLUE    {0, 0, 255}
RGB_YELLOW  {255, 255, 0}
RGB_CYAN    {0, 255, 255}
RGB_MAGENTA {255, 0, 255}
RGB_WHITE   {255, 255, 255}
RGB_OFF     {0, 0, 0}
```

## API Reference

### Inicjalizacja
```c
esp_err_t rgb_led_init(void);
```
Inicjalizuje RGB LED module. Musi być wywołane przed innymi funkcjami.

### Podstawowe Sterowanie
```c
// Ustawienie koloru RGB
esp_err_t rgb_led_set_color(rgb_color_t color);
esp_err_t rgb_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue);

// Wyłączenie LED
esp_err_t rgb_led_off(void);
```

### Animacje
```c
// Ustawienie trybu animacji
esp_err_t rgb_led_set_mode(rgb_led_mode_t mode, uint32_t period_ms);

// Zatrzymanie animacji
esp_err_t rgb_led_stop(void);
```

## Przykłady Użycia

### Podstawowe Mruganie
```c
#include "rgb_led.h"

void simple_blink_example(void) {
    // Inicjalizacja
    rgb_led_init();
    
    // Mruganie czerwonym kolorem co 500ms
    rgb_color_t red = RGB_RED;
    rgb_led_set_mode(RGB_MODE_BLINK, 500);
}
```

### Efekt Tęczy
```c
void rainbow_example(void) {
    rgb_led_init();
    
    // Szybki efekt tęczy (100ms między kolorami)
    rgb_led_set_mode(RGB_MODE_RAINBOW, 100);
}
```

### Flight Controller Status
```c
void flight_status_example(void) {
    rgb_led_init();
    
    // Wzorzec statusu flight controllera
    rgb_led_set_mode(RGB_MODE_FLIGHT_STATUS, 800);
    
    // Znaczenie kolorów:
    // Zielony: Armed/Ready
    // Niebieski: GPS Lock  
    // Żółty: Warning/Calibrating
    // Wyłączony: Standby
}
```

### Niestandardowy Kolor
```c
void custom_color_example(void) {
    rgb_led_init();
    
    // Ustawienie niestandardowego koloru fioletowego
    rgb_led_set_rgb(128, 0, 128);
    
    // Lub używając struktury
    rgb_color_t purple = {128, 0, 128};
    rgb_led_set_color(purple);
}
```

## Wzorce Flight Controller

### Status Indicators
- 🔴 **Czerwony**: Error/Disarmed
- 🟡 **Żółty**: Warning/Calibrating  
- 🔵 **Niebieski**: GPS Lock
- 🟢 **Zielony**: Armed/Ready
- 🌈 **Tęcza**: Test Mode/Celebration

### Typowe Wzorce
```c
// Błąd systemu
rgb_led_set_color((rgb_color_t)RGB_RED);

// Kalibracja
rgb_led_set_mode(RGB_MODE_BLINK, 300); // Żółte mruganie

// GPS Lock
rgb_led_set_color((rgb_color_t)RGB_BLUE);

// Armed i gotowy
rgb_led_set_color((rgb_color_t)RGB_GREEN);

// Tryb testowy
rgb_led_set_mode(RGB_MODE_RAINBOW, 50);
```

## Konfiguracja w Projekcie

### CMakeLists.txt
```cmake
idf_component_register(SRCS "hello_world_main.c"
                           "rgb_led.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_lcd esp_driver_gpio esp_driver_spi lvgl led_strip)
```

### idf_component.yml
```yaml
dependencies:
  lvgl/lvgl: ^8.3.0
  espressif/led_strip: ^2.5.3
```

## Thread Safety
- Module używa mutexu dla thread-safe operacji
- Animacje działają w osobnym task z priorytetem 5
- Maksymalny timeout dla mutex: 100ms

## Specyfikacje Techniczne
- **RMT frequency**: 10MHz
- **DMA**: Disabled (pojedynczy LED)
- **Max LEDs**: 1 (konfigurowalny)
- **Stack size**: 4096 bytes dla animation task
- **Priority**: 5 (konfigurowalny)

## Diagnostyka

### Debug Logs
Module używa ESP_LOG z tagiem "rgb_led":
```c
ESP_LOGI("rgb_led", "RGB LED initialized successfully");
ESP_LOGE("rgb_led", "Failed to create LED strip");
```

### Typowe Problemy
1. **LED nie świeci**: Sprawdź połączenie pinu 8 i zasilanie
2. **Nieprawidłowe kolory**: Sprawdź typ LED (WS2812 vs WS2811)
3. **Migotanie**: Sprawdź jakość zasilania i przewodów
4. **Brak animacji**: Sprawdź czy task został utworzony poprawnie

## Rozbudowa
Module można łatwo rozszerzyć o:
- Więcej LED w taśmie (zmiana max_leds)
- Nowe wzorce animacji
- Synchronizację z muzyką/dźwiękiem
- Komunikację z ground station
- Sensor-driven patterns 