# Tilt & Light Guard IoT (ESP32-S2 side)

Minimal ESP32-S2 project (PlatformIO + Arduino) for the TiltGuard system:
- Hosts a simple Wi‑Fi AP and web UI (ARM/DISARM/MUTE + 2 sliders)
- Talks to FRDM over UART with line-oriented ASCII commands
- Polls MPU‑6050 over I2C and periodically reports IMU_UPSIDEDOWN=0/1 to FRDM

## Hardware

ESP32‑S2‑Mini‑1U (adjust pins as needed):
- UART to FRDM: TX=GPIO18, RX=GPIO17 (change in `src/main.cpp` if different)
- I2C to MPU‑6050: SDA=GPIO33, SCL=GPIO35 (change in `src/main.cpp` if different)
- Power: 3.3 V to GY‑521 VCC, shared GND with FRDM

FRDM‑MCXC444:
- UART to ESP32: cross RX/TX, share GND
- LDR: node -> PTE21 (ADC0_SE4a), divider to 3.3 V/GND
- Tilt switch: PTC1 with pull‑up, IRQ on falling edge (or use on‑board SW3 PTA4)
- Green LED: PTD5 (TPM0 CH5) PWM
- Buzzer: PTE23 (GPIO) to active buzzer input

## Build & Flash (PlatformIO)

- Open this folder in VS Code (PlatformIO extension installed)
- `platformio.ini` uses `board = esp32s2dev` and framework Arduino
- If your board definition differs, change `board` to match your ESP32‑S2 variant

Upload and open serial monitor at 115200 baud. On boot:
- ESP creates AP `TiltGuard` (password `tiltguard123`)
- Visit `http://192.168.4.1/`
- Use buttons/sliders; the page polls `/status` every 500 ms

## Protocol

ESP32 → FRDM:
- `ARM\n`, `DISARM\n`, `MUTE\n`
- `TH_TILT=NNN\n`, `TH_LIGHT=NNN\n`
- `IMU_UPSIDEDOWN=0|1\n` (every ~500 ms)

FRDM → ESP32:
- `STATUS armed=0|1 alarm=0|1 thr_tilt=NNN thr_light=NNN light=NNN imuUD=0|1\n`
- `ALERT ON\n` / `ALERT OFF\n`

## Notes

- Pin defaults for ESP32‑S2 vary across modules; adjust `UART_TX_PIN`, `UART_RX_PIN`, `I2C_SDA_PIN`, `I2C_SCL_PIN`.
- If MPU‑6050 WHO_AM_I != 0x68, check wiring and that AD0 is tied to GND (0x68) or change address to 0x69.
- For STA mode instead of AP, replace `WiFi.mode(WIFI_AP)` + `WiFi.softAP(...)` with your SSID/PASS and wait for `WiFi.status()==WL_CONNECTED`.
