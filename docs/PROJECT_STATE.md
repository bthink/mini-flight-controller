# Mini Flight Controller - Project State

## Goal
- Display the nearest aircraft currently in flight near the configured location on the controller LCD.

## Current Behavior
- Device boots, initializes LCD + LVGL + WiFi.
- After WiFi connect, tracker fetches aircraft data from OpenSky.
- LCD shows:
  - WiFi status
  - nearest aircraft callsign, distance, altitude, heading
  - or `Aircraft: no data` when nothing is available.

## Implemented Components
- `main/wifi_manager.c`:
  - WiFi STA connect/retry/status API.
- `main/aircraft_tracker.c` + `main/aircraft_tracker.h`:
  - OpenSky integration:
    - OAuth2 client credentials flow (preferred)
    - Basic auth fallback (legacy)
  - Fetches `/api/states/all` with bbox from configured lat/lon/radius.
  - Filters out aircraft on ground (`on_ground == true`).
  - Selects nearest aircraft using haversine distance.
- `main/hello_world_main.c`:
  - UI integration for tracker data.
  - Tracker runs in dedicated FreeRTOS task.
  - LVGL loop stabilized (partial buffers, no full refresh, no separate `lvgl_tick` task).

## Configuration (menuconfig)
- `Flight Controller WiFi Configuration`:
  - `WIFI_SSID`, `WIFI_PASSWORD`, retry count.
- `Aircraft Tracker Configuration`:
  - `TRACKER_HOME_LAT`, `TRACKER_HOME_LON`
  - `TRACKER_RADIUS_KM` (currently expected to be 50 km)
  - `TRACKER_REFRESH_SEC`
  - OAuth2:
    - `TRACKER_OPENSKY_CLIENT_ID`
    - `TRACKER_OPENSKY_CLIENT_SECRET`
    - `TRACKER_OPENSKY_TOKEN_URL`
  - Optional legacy:
    - `TRACKER_OPENSKY_USERNAME`
    - `TRACKER_OPENSKY_PASSWORD`

## Important Technical Decisions
- App partition increased (large single app) because firmware exceeded 1 MB.
- Stack issues fixed:
  - moved large OAuth/JSON buffers from stack to heap.
  - increased tracker task stack.
- HTTP header buffer issue fixed by setting larger per-request buffers in `esp_http_client_config_t`.

## Known Limitations
- Source endpoint (`states/all`) does not provide aircraft model/type directly.
- Route origin/destination is not available from `states/all`; requires additional lookup endpoints.
- Token is currently fetched per tracker cycle; no token cache yet.

## Security Notes
- Sensitive credentials are in local `sdkconfig` (ignored by git by default).
- Do not commit secrets in source files.

## Run / Flash Quickstart
```bash
cd /Users/bartoszfink/dzikieProjekty/mini-flight-controller
. ~/esp/esp-idf/export.sh
idf.py -p /dev/cu.usbmodem101 flash
idf.py -p /dev/cu.usbmodem101 monitor
```

## Typical Troubleshooting
- If flashing fails on wrong port:
  - always pass `-p /dev/cu.usbmodem101` (or currently detected modem port).
- If `No serial data received`:
  - BOOT hold -> RESET tap -> release BOOT, retry flash.
- If tracker shows no aircraft:
  - verify OAuth2 client credentials,
  - verify location/radius,
  - check tracker logs for HTTP status codes.
