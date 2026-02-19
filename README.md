# Mini Flight Controller (ESP32-C6)

Firmware dla kontrolera z ekranem LCD, który:
- łączy się z Wi-Fi,
- pobiera najbliższy samolot w okolicy,
- wyświetla dane lotu na ekranie,
- steruje podświetleniem LCD i RGB LED zależnie od tego, czy samolot został wykryty.

Projekt oparty o ESP-IDF + LVGL.

## Co zobaczysz na ekranie

Gdy samolot jest dostępny:
- `Aircraft: <callsign>`
- `Type: <kod + nazwa typu>` (best-effort)
- `Dist / Alt / Head`
- `Route: <origin -> destination>` (best-effort)

Gdy samolotu nie ma:
- status Wi-Fi (`Connecting / Connected / Failed / Disconnected`)
- `Aircraft: no data` lub `unavailable`

## Wymagania

- ESP-IDF (projekt budowany na target `esp32c6`)
- Python i narzędzia ESP-IDF (`idf.py`)
- płytka z ESP32-C6 i ekranem SPI ST7789 podłączonym wg pinów w `main/config.h`
- połączenie internetowe do API:
  - OpenSky (`opensky-network.org`)
  - ADS-B DB (`api.adsbdb.com`) dla enrichmentu typu/trasy

## Szybki start

### 1. Wejdź do projektu

```bash
cd /path/to/mini-flight-controller
```

### 2. Załaduj środowisko ESP-IDF

```bash
. ~/esp/esp-idf/export.sh
```

### 3. (Opcjonalnie, pierwszy raz) ustaw target

```bash
idf.py set-target esp32c6
```

### 4. Zbuduj projekt

```bash
idf.py build
```

### 5. Wgraj firmware

```bash
idf.py -p /dev/cu.usbmodem101 flash
```

### 6. Uruchom monitor logów

```bash
idf.py -p /dev/cu.usbmodem101 monitor
```

Wyjście z monitora: `Ctrl + ]`.

## Konfiguracja (Wi-Fi, lokalizacja, API)

Najwygodniej przez menuconfig:

```bash
idf.py menuconfig
```

Sekcje:
- `Flight Controller WiFi Configuration`
  - `WIFI_SSID`
  - `WIFI_PASSWORD`
  - retry/timeout
- `Aircraft Tracker Configuration`
  - `TRACKER_HOME_LAT`, `TRACKER_HOME_LON` (punkt odniesienia)
  - `TRACKER_RADIUS_KM`
  - `TRACKER_REFRESH_SEC`
  - OpenSky OAuth2:
    - `TRACKER_OPENSKY_CLIENT_ID`
    - `TRACKER_OPENSKY_CLIENT_SECRET`
    - `TRACKER_OPENSKY_TOKEN_URL`
  - fallback legacy:
    - `TRACKER_OPENSKY_USERNAME`
    - `TRACKER_OPENSKY_PASSWORD`

Aktualny domyślny punkt projektu to Ołtaszyn (Wrocław) w `sdkconfig`.

## Hardware / piny

Definicje pinów są w `main/config.h`.

Najważniejsze:
- LCD SPI: `SCLK=7`, `MOSI=6`, `DC=15`, `RST=21`, `CS=14`
- Backlight LCD: `GPIO22` (sterowanie PWM)
- RGB LED: `GPIO8`

## Zachowanie LED/backlight

- RGB LED:
  - `ON` (zielony), gdy wykryto samolot
  - `OFF`, gdy brak samolotu
- Podświetlenie LCD:
  - ~`20%` gdy brak samolotu
  - `100%` gdy samolot wykryty

## Źródła danych i ograniczenia

- Podstawowe dane pozycyjne pochodzą z OpenSky `/api/states/all`.
- `Type` i `Route` są wzbogacane best-effort:
  - najpierw z `api.adsbdb.com`,
  - fallback dla trasy z OpenSky `/api/flights/aircraft`.
- Brak typu/trasy dla części lotów jest normalny (zależność od dostępności danych zewnętrznych).

## Najczęstsze problemy

### `zsh: command not found: idf.py`

Niezaładowane środowisko ESP-IDF:

```bash
. ~/esp/esp-idf/export.sh
```

### Flash nie działa / zły port

- sprawdź poprawny port i podaj go jawnie przez `-p`
- przykład:

```bash
idf.py -p /dev/cu.usbmodem101 flash
```

### `No serial data received`

Spróbuj sekwencji:
1. Przytrzymaj `BOOT`
2. Krótko naciśnij `RESET`
3. Puść `BOOT`
4. Ponów flash

### Samoloty się nie pojawiają

- sprawdź Wi-Fi i internet,
- zweryfikuj credentials OpenSky,
- sprawdź `TRACKER_HOME_LAT/LON` i `TRACKER_RADIUS_KM`,
- sprawdź logi (`idf.py monitor`) pod kątem kodów HTTP.

## Struktura projektu

- `main/hello_world_main.c` - LCD/LVGL/UI + pętla główna
- `main/aircraft_tracker.c` - pobieranie i enrichment danych lotniczych
- `main/wifi_manager.c` - obsługa Wi-Fi STA
- `main/rgb_led.c` - sterowanie RGB LED
- `main/config.h` - piny i stałe
- `docs/PROJECT_STATE.md` - szczegółowy stan techniczny projektu

## Bezpieczeństwo

- Sekrety (SSID, hasła, tokeny) są w lokalnym `sdkconfig`.
- Nie commituj sekretów do repozytorium.
