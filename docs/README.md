# Dokumentacja ESP32-C6 Mini Flight Controller

## Struktura Dokumentacji

### 📚 Get Started
- `get-started/index.rst` - Główny przewodnik ESP-IDF
- `get-started/esp32c6_output_log.inc` - Logi specyficzne dla ESP32-C6
- `get-started/linux-macos-setup.rst` - Instalacja na macOS/Linux
- `get-started/establish-serial-connection.rst` - Konfiguracja połączenia szeregowego

### 🔧 API Reference - Peripherals
- `api-reference/peripherals/lcd/` - Dokumentacja LCD (ST7789)
- `api-reference/peripherals/spi_master.rst` - SPI Master (dla LCD i sensorów)
- `api-reference/peripherals/gpio.rst` - GPIO Control
- `api-reference/peripherals/mcpwm.rst` - Motor Control PWM (ESC)
- `api-reference/peripherals/i2c.rst` - I2C (IMU, sensory)
- `api-reference/peripherals/uart.rst` - UART (GPS, telemetria)
- `api-reference/peripherals/adc_oneshot.rst` - ADC (sensory analogowe)
- `api-reference/peripherals/rmt.rst` - RMT (DShot protocol)
- `api-reference/peripherals/ledc.rst` - LEDC (servo PWM)

### 📖 Examples
#### Basic Examples
- `examples/get-started/blink/` - Podstawowy przykład migania LED
- `examples/get-started/hello_world/` - Hello World

#### Peripheral Examples
- `examples/peripherals/lcd/spi_lcd_touch/` - **SPI LCD z ST7789 + LVGL**
- `examples/peripherals/mcpwm/mcpwm_servo_control/` - **Sterowanie servo**
- `examples/peripherals/rmt/dshot_esc/` - **DShot ESC control**

### 🔩 Hardware Reference
- `hardware/hw-reference.rst` - Dokumentacja sprzętowa ESP32-C6

### 📋 Compatibility & Reference
- `COMPATIBILITY.md` - Kompatybilność chipów ESP z ESP-IDF
- `ESP-IDF_README.md` - Główny README ESP-IDF

## Kluczowe Informacje dla Flight Controller

### Motor Control
- **MCPWM**: Do sterowania ESC w trybie standardowym PWM
- **RMT**: Do protokołu DShot (cyfrowy protokół ESC)
- **LEDC**: Do sterowania servo (gimbal, kontrole lotu)

### Sensors & Communication
- **I2C**: IMU (MPU6050/9250), magnetometr, barometr
- **SPI**: Dodatkowe sensory (barometr, flash memory)
- **UART**: GPS, telemetria, debugging
- **ADC**: Monitoring baterii, sensory analogowe

### Display & UI
- **SPI LCD**: ST7789 controller z LVGL dla GUI
- **GPIO**: Przyciski, LED statusu, buzzer

### Connectivity
- **WiFi 6**: Telemetria, OTA updates
- **Bluetooth LE**: Konfiguracja, debugging

## Następne Kroki
1. Przejrzyj przykład `spi_lcd_touch` dla implementacji ST7789
2. Sprawdź `mcpwm_servo_control` i `dshot_esc` dla sterowania silnikami
3. Wykorzystaj dokumentację I2C dla podłączenia IMU
4. Zapoznaj się z GPIO dla pin mappingu 