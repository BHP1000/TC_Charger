# TC_Charger -- Open Source EV Charger Controller

An ESP32-S3 based controller for the **HK-LF-115-58 6.6kW On-Board Charger (OBC)** by TC Charger Standard. This project provides full CAN bus control, monitoring, and telemetry for charging lithium battery packs from 20V to 108V.

Originally designed for a 20S (84V) electric scooter build, but configurable for any pack voltage within the charger's range.

## License

**CC BY-NC-SA 4.0** -- Free for personal and open-source use. Commercial use requires a separate license. See [LICENSE](LICENSE) for full terms.

All contributions must be open source. If you want commercial rights, contact the project owner and contribute your improvements back.

---

## Hardware

### Charger
| Item | Detail |
|------|--------|
| Model | HK-LF-115-58 |
| Configuration | 9000/HD623 |
| Manufacturer | TC Charger Standard |
| Power | 6.6 kW |
| Output | 20V - 108V, up to 60A |
| CAN bus | 250 kbps, CAN 2.0B extended 29-bit frames (SAE J1939 style addressing) |
| Charging method | Pulse charging (CC-CV with pulsed current delivery) |

### Controller (Main Brain)
| Item | Detail |
|------|--------|
| MCU | ESP32-S3 DevKitC-1 N16R8 |
| Flash | 16 MB |
| PSRAM | 8 MB OPI |
| Framework | Arduino (single .ino sketch for Arduino IDE) |
| Upload | COM3 @ 921600 baud |
| Monitor | 115200 baud (Putty or any serial terminal) |

### CAN Transceiver
| Item | Detail |
|------|--------|
| Chip | SN65HVD230 (WCMCU-230) or CJMCU-1051 (TJA1051) |
| Voltage | **3.3V only** -- both transceivers work at 3.3V. Do NOT power at 5V (will damage ESP32 GPIO) |
| TX GPIO | 4 |
| RX GPIO | 5 |
| S pin | Must be tied to **GND** for normal operation |
| Termination | 120 ohm across CANH/CANL at each bus end |

### Daughter Board (Temp Node) -- Optional
| Item | Detail |
|------|--------|
| MCU | ESP32-C3 Super Mini |
| Purpose | 5x NTC thermistor array + AHT20/BMP280 environment sensor |
| Communication | RS485 Modbus to Main Brain (GPIO 17/18) |
| NTC sensors | 100K 3950 beta, channels 0-4 (GPIO 0-4 on C3) |

### Additional Peripherals (Main Brain)
| Peripheral | Interface | Pins |
|---|---|---|
| SH1106 OLED 128x64 | I2C | SDA=GPIO8, SCL=GPIO9, addr 0x3C |
| AHT20 temp/humidity | I2C | addr 0x38 (shared bus) |
| BMP280 pressure | I2C | addr 0x76 or 0x77 (shared bus) |
| QMC5883L compass | I2C | addr 0x0D (shared bus) |
| HGLRC M100 GPS | UART2 | RX=GPIO15, TX=GPIO16 @ 115200 |
| WS2812B RGB LED | Data | GPIO48 (onboard) |
| NEXT button | Digital | GPIO6 (active LOW, internal pullup) |
| HOLD button | Digital | GPIO7 (active LOW, internal pullup) |

---

## CAN Protocol

### Frame 1: Controller to Charger (TX)
- **ID:** `0x1806E5F4` (extended 29-bit)
- **Interval:** 1000 ms heartbeat (charger disables if no frame for 5 seconds)
- **Byte order:** Big-endian (Motorola)

| Byte | Field | Encoding |
|------|-------|----------|
| 0-1 | Max charge voltage | 0.1 V/bit (e.g. 840 = 84.0V) |
| 2-3 | Max charge current | 0.1 A/bit (e.g. 200 = 20.0A) |
| 4 | Control | 0x00=charge, 0x01=disable, 0x02=sleep |
| 5 | Mode | 0x00=charging, 0x01=heating |
| 6-7 | Reserved | 0x00 |

### Frame 2: Charger to Controller (RX)
- **ID:** `0x18FF50E5` (extended 29-bit)
- **Interval:** 1000 ms

| Byte | Field | Encoding |
|------|-------|----------|
| 0-1 | Output voltage | 0.1 V/bit |
| 2-3 | Output current | 0.1 A/bit |
| 4 | Fault flags | See below |
| 5 | Status flags | See below |
| 6 | Connector flags | See below |
| 7 | Temperature | raw - 40 = degrees C |

### Fault Flags (Byte 4)
| Bit | Field |
|-----|-------|
| 0 | Hardware fault |
| 1 | Temperature fault |
| 2-3 | Input voltage (0=OK, 1=under, 2=over, 3=none) |
| 4 | Output under-voltage |
| 5 | Output over-voltage |
| 6 | Output over-current |
| 7 | Output short circuit |

### Status Flags (Byte 5)
| Bit | Field |
|-----|-------|
| 0 | Communication timeout |
| 1-2 | Working status (0=init, 1=working, 2=stopped, 3=standby) |
| 3 | Initialization complete |
| 4 | Fan on |
| 5 | Cooling pump on |

---

## Serial Commands

All commands are single keystrokes (no Enter needed). Works via USB serial (Putty/Arduino Serial Monitor at 115200 baud) and also via BLE and WiFi.

| Key | Command | Description |
|-----|---------|-------------|
| `r` | Resume | Start charging at current voltage/current setpoints |
| `s` | Stop | Disable output, charger stays awake on CAN |
| `e` | End | Charger enters sleep mode. Wake with `w` then `r` |
| `w` | Wake | Send CAN handshake frame after power-on or sleep |
| `+` | Current +1A | Increase charge current (max 60A) |
| `-` | Current -1A | Decrease charge current (min 0A) |
| `v` | Voltage +0.5V | Increase target voltage (max 108V) |
| `V` | Voltage -0.5V | Decrease target voltage (min 20V) |
| `b` | Buddy Mode | Charge another battery -- stops output, unlocks full voltage range |
| `n` | Normal Mode | Exit buddy mode, restore home settings (84V/2A) |
| `0` | Reset Energy | Reset session and previous counters (lifetime preserved) |
| `p` | Print Status | Full report: setpoints, charger output, energy, range estimates |
| `?` | Help | Show command list |

### Boot Defaults
- Voltage: **84.0V** (20S x 4.20V)
- Current: **2.0A** (safe starting level)
- State: **DISABLED** (nothing happens until you type `r`)

---

## Energy Tracking

Three tiers of energy measurement, plus separate buddy tracking:

| Tier | Scope | Persistence |
|------|-------|-------------|
| **Session** | Current charge event (Wh, Ah, duration, miles added) | RAM -- resets on new session |
| **Previous** | Last completed charge session | RAM -- overwritten each time |
| **Lifetime** | Total energy ever charged | NVS flash -- survives reboots |
| **Buddy** | External battery charge only | RAM -- resets when exiting buddy mode |

Energy is calculated from EMA-smoothed voltage and current values, which accurately handles the charger's pulse charging behavior.

### Range Estimation
- Default efficiency: **32 Wh/mile** (adjustable)
- Status report (`p`) shows charge time needed for +5, +10, +20 miles at current rate
- Every status line shows miles added this session

---

## Remote Control

### BLE (Bluetooth Low Energy)
- Device name: `ScooterCharger`
- Service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- Command characteristic (`...26b0`): Write single-char commands (r, s, e, w, +, -, v, V, b, n, 0, p, ?)
- Charger telemetry (`...26b1`): 22-byte binary, notified at 1 Hz

**Telemetry format (little-endian):**
| Offset | Size | Field |
|--------|------|-------|
| 0 | 2 | Smoothed voltage (dV, uint16) |
| 2 | 2 | Smoothed current (dA, uint16) |
| 4 | 4 | Session Wh (float) |
| 8 | 4 | Lifetime Wh (float) |
| 12 | 1 | Charging (0/1) |
| 13 | 1 | Buddy mode (0/1) |
| 14 | 4 | Power watts (float) |
| 18 | 4 | Miles added (float) |

### WiFi (Fallback)
- AP SSID: `ScooterCharger`
- Password: `charge84v`
- Status: `http://192.168.4.1/status` (JSON)
- Commands: `http://192.168.4.1/cmd?c=r` (any single-char command)

WiFi runs as a soft AP simultaneously with BLE. Use BLE when signal is strong, fall back to WiFi when needed.

---

## OLED Display

7 pages on the SH1106 128x64 OLED, auto-advancing every 8 seconds:

| Page | Title | Content |
|------|-------|---------|
| 1/7 | Charger Output | Pack voltage, set voltage, current, power, faults |
| 2/7 | Ambient Sensors | Main board + daughter board temp/humidity/pressure |
| 3/7 | Battery NTC Temps | 5x NTC thermistor readings (Fahrenheit) |
| 4/7 | Pack Status | BMS voltage, current, SOC, Ah, charge/discharge flags |
| 5/7 | Pack Health | MOS temp, battery temps, SOH, cycles, runtime |
| 6/7 | 20S Cell Volts | All 20 cell voltages in 5x4 grid + min/max/delta |
| 7/7 | GPS & Range | Lat/lon, speed (mph), range (miles), compass, satellites |

NEXT button (GPIO 6) advances pages. HOLD button (GPIO 7) freezes on current page.

---

## RGB LED Status

The onboard WS2812B LED (GPIO 48) indicates system state:

| Color | Pattern | Meaning |
|-------|---------|---------|
| Blue | Slow blink | Idle, charger not active |
| Green | Solid | Charging |
| Red | Fast blink | Overtemp or fault |
| Cyan | Blink | Undertemp (below freezing) |
| Orange | Blink | BMS fault |
| Yellow | Blink | Communication timeout |
| White | Fast blink | Booting |

---

## Pulse Charging Behavior

The HK-LF-115-58 uses pulse charging. CAN status frames report **instantaneous** snapshots, which bounce wildly between 0A and 30+ A even when the average is 12A. The firmware uses exponential moving average (EMA) smoothing with alpha=0.1 to display the true average values.

All energy calculations use smoothed values, which accurately reflect total energy delivered regardless of pulse vs. steady delivery.

---

## Hardware Requirements

### Minimum ESP32-S3 Board Specs

| Requirement | Minimum | This Project Uses |
|---|---|---|
| MCU | ESP32-S3 | ESP32-S3 DevKitC-1 |
| Flash | **16 MB** | N16R8 (16MB Flash / 8MB PSRAM) |
| PSRAM | 8 MB OPI | N16R8 |
| USB | USB-to-UART bridge (CP2102/CH340) | DevKitC-1 has both native USB + UART bridge |

**Important:** Boards with only 4MB flash (like the basic ESP32-S3-WROOM-1 N4) will NOT work. The compiled firmware is ~1.3MB and requires a 3MB APP partition which needs 16MB flash. Verify your board's flash size before purchasing.

Common compatible boards:
- ESP32-S3 DevKitC-1 **N16R8** (recommended, this is what we use)
- ESP32-S3 DevKitC-1 N8R8 (8MB flash -- will work with smaller partition but tight)
- Any ESP32-S3 board with 16MB+ flash and exposed GPIO 4, 5, 6, 7, 8, 9, 15, 16, 17, 18, 48

---

## Arduino IDE Setup

Follow these steps exactly. Incorrect settings will cause boot loops, blank serial output, or flash errors.

### Step 1: Install ESP32 Board Package

1. Open Arduino IDE
2. Go to **File > Preferences**
3. In "Additional Board Manager URLs" add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools > Board > Boards Manager**
5. Search for "esp32" and install **"esp32 by Espressif Systems"**

### Step 2: Board Settings (Tools Menu)

Set each of these in the **Tools** menu. Every setting matters.

| Setting | Value | Why |
|---|---|---|
| **Board** | ESP32S3 Dev Module | Matches our MCU |
| **Flash Size** | **16MB (128Mb)** | Must match your board's physical flash. Using 4MB on a 16MB board causes boot loops with "partition table error" |
| **Partition Scheme** | **16M Flash (3MB APP/9.9MB FATFS)** | Firmware is ~1.3MB, default 1.2MB partition is too small. This gives 3MB for the app |
| **PSRAM** | **OPI** | N16R8 uses OPI PSRAM. Wrong setting = crash or no PSRAM |
| **USB CDC On Boot** | **Disabled** | Critical. If "Enabled", Serial output goes to native USB instead of UART bridge (COM3). You'll see nothing in Putty/Serial Monitor. |
| **Upload Speed** | 921600 | Fastest reliable speed |
| **USB Mode** | Hardware CDC and JTAG | Default, leave as-is |
| **Port** | COM3 (or your board's port) | Check Device Manager if unsure |

### Step 3: Install Libraries

Go to **Sketch > Include Library > Manage Libraries** and install:

| Library | Search Term | Author |
|---|---|---|
| FastLED | FastLED | Daniel Garcia |
| U8g2 | U8g2 | oliver |
| Adafruit AHTX0 | Adafruit AHTX0 | Adafruit |
| Adafruit BMP280 Library | Adafruit BMP280 | Adafruit |
| TinyGPSPlus | TinyGPSPlus | Mikal Hart |

Accept any dependency prompts (Adafruit BusIO, Adafruit Unified Sensor, etc.)

### Step 4: Upload

1. Open `TC_Charger_MainBrain/TC_Charger_MainBrain.ino`
2. Select your port under **Tools > Port**
3. Click **Upload** (arrow button)
4. Open Serial Monitor or Putty at **115200 baud**
5. Press the **RST** button on the board
6. You should see the boot banner and status messages

### Troubleshooting

**Boot loop with "partition table error":**
```
E (29) flash_parts: partition 3 invalid - offset 0x310000 size 0x300000 exceeds flash chip size 0x400000
```
This means Flash Size is wrong (set to 4MB instead of 16MB) or a previous upload left a bad partition table.
Fix:
1. **Tools > Flash Size > 16MB (128Mb)**
2. **Tools > Erase All Flash Before Sketch Upload > Enabled**
3. Upload the sketch
4. After successful boot, set **Erase All Flash** back to **Disabled** (so NVS settings are preserved on future uploads)

**Blank serial output (nothing in Putty):**
- Check **Tools > USB CDC On Boot** is set to **Disabled**
- Make sure Putty is on the correct COM port at 115200 baud
- Press RST on the board after opening Putty
- In Putty settings: Terminal > check "Implicit CR in every LF" (fixes staircase text)

**Sketch too big / exceeds available space:**
- Check **Tools > Partition Scheme** is set to **"16M Flash (3MB APP/9.9MB FATFS)"**
- The default 1.2MB APP partition is too small for this firmware

**Upload fails / can't connect:**
- Try holding the **BOOT** button while clicking Upload, release after "Connecting..." appears
- Check that no other program (Putty, Serial Monitor) has the COM port open during upload
- Try a different USB cable (some are charge-only, no data)

---

## Wiring Summary

### ESP32-S3 Main Brain Pin Map
| GPIO | Function |
|------|----------|
| 4 | CAN TX (to SN65HVD230 TXD) |
| 5 | CAN RX (from SN65HVD230 RXD) |
| 6 | NEXT button (active LOW, pullup) |
| 7 | HOLD button (active LOW, pullup) |
| 8 | I2C SDA (OLED, AHT20, BMP280, QMC5883L) |
| 9 | I2C SCL (shared bus) |
| 15 | GPS RX (from GPS module TX) |
| 16 | GPS TX (to GPS module RX) |
| 17 | RS485 TX (to RS485 DI) |
| 18 | RS485 RX (from RS485 RO) |
| 48 | WS2812B RGB LED (onboard) |

### CAN Bus Wiring
```
ESP32-S3 GPIO4 ---> SN65HVD230 TXD
ESP32-S3 GPIO5 <--- SN65HVD230 RXD
SN65HVD230 VCC ---> 3.3V
SN65HVD230 GND ---> GND
SN65HVD230 S   ---> GND (normal mode)
SN65HVD230 CANH --> Charger CANH --[120R]--> CANL
SN65HVD230 CANL --> Charger CANL
```

Healthy idle bus: CANH-CANL differential near 0V.

---

## Contributing

Contributions are welcome under the CC BY-NC-SA 4.0 license:

1. Fork the repository
2. Create a feature branch
3. Submit a pull request with clear description
4. All contributions become open source under the same license

For commercial licensing inquiries, open an issue or contact the project owner.

---

## Disclaimer

This project involves high-voltage battery charging. Incorrect wiring, configuration, or use can cause fire, explosion, property damage, or injury. Use at your own risk. The authors assume no liability for any damages resulting from the use of this project.

Always verify your pack voltage, current limits, and wiring before enabling charging. Never leave a charging battery unattended.
