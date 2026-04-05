# TC_Charger Module — Agent Identity & Knowledge Base

## Identity

You are the **TC_Charger Agent** — the authoritative intelligence for the 20S (84V) smart charging
system. Your domain is everything related to controlling, monitoring, protecting, and integrating
the EV on-board charger (OBC) into the scooter ecosystem. You have complete working knowledge of
every file in this module and are the final authority on charger control decisions.

This CLAUDE.md is loaded automatically whenever `c:\Scooter_Development\TC_Charger` is the working
directory. Cross-conversation persistent memory lives at:
`C:\Users\MSI\.claude\projects\c--Scooter-Development\memory\`

---

## Project Context

- **System**: 20S Lithium pack — nominal 84V (4.2V × 20S), min ~72V (3.6V × 20S), max 84V
- **Charger**: HK-LF-115-58 6.6 kW 108V/60A OBC (TC Standard OBC) — CAN-controlled
- **Main Brain**: ESP32-S3 DevKitC-1 N16R8 — PlatformIO env `main_brain_s3`, build flag `MAIN_BRAIN`
- **Temp Node / Daughter Board**: ESP32-C3 Super Mini — PlatformIO env `temp_node_c3`, build flag `TEMP_NODE`
- **Upload ports**: Main Brain = COM3, Temp Node = COM4
- **Framework**: Arduino via PlatformIO

---

## Hardware — CAN Bus

| Item | Detail |
|------|--------|
| Transceiver (production) | WCMCU-230 (SN65HVD230) — 3.3V device |
| Transceiver (test bench) | SN65HVD1051 module — VCC→3.3V, S pin→GND |
| Production CAN TX GPIO | GPIO 4 (Main Brain) |
| Production CAN RX GPIO | GPIO 5 (Main Brain) |
| Test sketch CAN TX GPIO | GPIO 8 |
| Test sketch CAN RX GPIO | GPIO 9 |
| Baud rate | 250 kbps |
| Frame type | Extended 29-bit frames only |
| Termination | 120 Ω across CANH/CANL at each bus end |
| S pin on transceiver | Must be tied to GND for normal operation; VCC = listen-only/silent |

**Healthy idle bus voltage**: ~2.5V measured CANH to CANL.
Readings below 2V indicate the charger CAN controller is not active or bus wiring fault.

---

## CAN Protocol — Complete Frame Specification

### Frame 1: BMS → Charger (we send)
- **ID**: `0x1806E5F4` (extended 29-bit)
- **Cycle**: 1000 ms (must send at least every 5000 ms or charger disables output)
- **Byte order**: Motorola / big-endian (MSB in lower byte index)

| Byte | Field | Encoding |
|------|-------|----------|
| 1–2 | Max allowable charging voltage | 0.1 V/bit, offset 0. e.g. 840 = 84.0V |
| 3–4 | Max allowable charging current | 0.1 A/bit, offset 0. e.g. 200 = 20.0A |
| 5 | Control / work enablement | 0x00=charge, 0x01=close output, 0x02=end+sleep |
| 6 | Operating mode | 0x00=charging, 0x01=heating mode |
| 7 | Reserved | 0x00 |
| 8 | Reserved | 0x00 |

**Control byte values**:
- `0x00` — Charger starts/continues charging output
- `0x01` — Charger closes output immediately (no sleep)
- `0x02` — Charge end: charger closes output and goes to sleep (requires new wake via AC cycle)
- Any other value — ignored / invalid

**Heating mode** (`BYTE6 = 0x01`): Charger drives heating film inside battery pack. Charger does
not monitor output voltage in this mode. Heating film resistance must be designed for battery max
voltage. Always close output before switching modes.

### Frame 2: Charger → BMS (we receive)
- **ID**: `0x18FF50E5` (extended 29-bit)
- **Cycle**: 1000 ms
- **Byte order**: Motorola / big-endian

| Byte | Field | Encoding |
|------|-------|----------|
| 1–2 | Output voltage | 0.1 V/bit, offset 0 |
| 3–4 | Output current | 0.1 A/bit, offset 0 |
| 5 | Fault flags | See bit map below |
| 6 | Status flags | See bit map below |
| 7 | Connector / lock flags | See bit map below |
| 8 | Temperature | raw − 40 = °C (e.g. 0 = −40°C, 65 = 25°C) |

**BYTE 5 — Fault flags**:
| Bit | Field | Values |
|-----|-------|--------|
| 0 | Hardware protection | 0=Normal, 1=Fault |
| 1 | Temperature protection | 0=Normal, 1=Internal temp fault |
| 2–3 | Input voltage status | 0=Normal, 1=Under-voltage, 2=Over-voltage, 3=No input |
| 4 | Output under-voltage | 0=Normal, 1=Fault |
| 5 | Output over-voltage | 0=Normal, 1=Fault |
| 6 | Output over-current | 0=Normal, 1=Fault |
| 7 | Output short circuit | 0=Normal, 1=Fault |

**BYTE 6 — Status flags**:
| Bit | Field | Values |
|-----|-------|--------|
| 0 | Communication status | 0=Normal, 1=Receive timeout |
| 1–2 | Working status | 0=Undefined, 1=Working, 2=Stopped, 3=Stop/Standby |
| 3 | Initialization complete | 0=Not complete, 1=Complete |
| 4 | Fan working | 0=Off, 1=On |
| 5 | Cooling pump working | 0=Off, 1=On (turns on >60°C, stays on ≥5 min after, then off at <55°C) |

**BYTE 7 — Connector / lock flags**:
| Bit | Field | Values |
|-----|-------|--------|
| 0–1 | CC signal status | 0=Not connected, 1=Half connected, 2=Normal, 3=Resistance error |
| 2 | CP signal status | 0=No CP signal, 1=CP normal |
| 3 | Charging socket overheat | 0=Normal, 1=Over-temp protection |
| 4–6 | Electronic lock state | 0=Judging, 1=Locked, 2=Unlocked, 3=Unlock fault, 4=Lock fault |
| 7 | S2 switch status | 0=Open, 1=Closed |

### Charger Sleep / Wake Behaviour
- Charger wakes automatically when AC power is applied
- If no CAN frame received within **5 seconds**, charger disables output
- `BYTE5 = 0x02` (sleep command) puts charger into deep sleep — requires AC power cycle to wake
- `BYTE5 = 0x01` (disable) closes output but charger stays awake and listening
- Keep-alive: send frame every **1000 ms** (production) or **3000 ms** (test bench)

---

## Production Firmware Architecture

### File Structure
```
TC_Charger/
├── src/
│   ├── main.cpp                    — setup/loop, driver init sequence
│   └── drivers/
│       ├── can_driver.h/.cpp       — TWAI CAN peripheral, charger command/status
│       ├── safety_state_machine.h/.cpp — charge safety FSM
│       ├── thermal_logic.h/.cpp    — NTC temp derating, overtemp/undertemp
│       ├── rs485_driver.h/.cpp     — RS485 Modbus to JK-BMS and Temp Node
│       ├── jk_bms.h/.cpp           — JK-BMS register parsing
│       ├── ntc_array.h/.cpp        — 6-channel NTC thermistor ADC
│       ├── oled_display.h/.cpp     — SH1106 OLED 7-page display
│       ├── ble_server.h/.cpp       — BLE server with 8 characteristics
│       ├── gps_driver.h/.cpp       — TinyGPSPlus + QMC5883L compass
│       ├── button_driver.h/.cpp    — NEXT/HOLD buttons, debounce
│       ├── bme280_driver.h/.cpp    — AHT20+BMP280 env sensor
│       └── rgb_led.h/.cpp          — WS2812B GPIO48 status LED
├── test/
│   └── charger_can_test/
│       └── charger_can_test.ino   — standalone Arduino test sketch
├── TC_Charger_Documentation/
│   ├── TC Standard OBC CAN Protocal .xlsx — charger CAN spec
│   └── HK-LF-115-58 6.6KW108V60A ... .pdf — charger hardware datasheet
└── JK_BMS/
    └── JK_BMS_RS485_Modbus.pdf    — BMS Modbus register map
```

### can_driver.cpp — Key Behaviours
- GPIO 4 (TX) / GPIO 5 (RX), 250 kbps, extended frames, TWAI_MODE_NORMAL
- TWAI filter set to accept only `0x18FF50E5`; falls back to accept-all if filter install fails
- `can_driver_update()` called every loop: sends heartbeat every 1000 ms, drains RX queue
- `s_last_cmd` persists last command — heartbeat always re-sends last known command
- Status marked stale (`.valid = false`) if no RX for >5000 ms
- Serial log format: `[CAN] RX: V A fault=0xXX stat=0xXX temp=°C`

### safety_state_machine.cpp — States
```
INIT → IDLE → CHARGING
                ↓
           OVERTEMP / UNDERTEMP / BMS_FAULT / COMM_TIMEOUT → FAULT
```
- `safety_sm_update()` called every loop
- `safety_sm_report_bms_fault(bool)` / `safety_sm_report_comm_timeout(bool)` feed into state

### thermal_logic.cpp — Derating Curve (LUT)
The current limit is a lookup-table derate based on max NTC temperature:

| Temp °C | Max Current (dA) | Max Current (A) |
|---------|-----------------|-----------------|
| 0 | 0 | 0.0 — inhibited |
| 5 | 50 | 5.0 |
| 10 | 100 | 10.0 |
| 15 | 150 | 15.0 |
| 20 | 200 | 20.0 |
| 25 | 200 | 20.0 — peak |
| 30 | 180 | 18.0 |
| 35 | 160 | 16.0 |
| 40 | 130 | 13.0 |
| 42 | 100 | 10.0 |
| 44 | 80 | 8.0 |
| 46 | 60 | 6.0 |
| 48 | 40 | 4.0 |
| 50 | 20 | 2.0 |
| 52 | 10 | 1.0 |
| 55 | 0 | 0.0 — cutoff |

- Overtemp cutoff: 55°C — sends `CAN_CTRL_DISABLE`
- Undertemp inhibit: 0°C — no charging below freezing
- Temperature source: 6-channel NTC array via Temp Node RS485

### NTC Array (ntc_array.cpp)
- 6 channels, GPIO 0–5 (ADC), 100K NTC 3950
- Per-channel measured series resistors: `{98300, 98500, 98900, 98900, 98900, 98900}` Ω
- Beta = 3950, reference 25°C

### OLED Display (oled_display.cpp)
- SH1106 128×64, I2C GPIO 8 (SDA) / GPIO 9 (SCL), address 0x3C
- 7 pages: Overview, Temps, BMS Pack, BMS Health, BMS Cells, Env, GPS
- Header bar: page title + `[CHG]` charging indicator + `[HLD]` hold indicator
- NEXT button (GPIO 6): advance page; HOLD button (GPIO 7): toggle display freeze

### BLE Server (ble_server.cpp)
Service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

| Characteristic | UUID suffix | Size | Content |
|----------------|-------------|------|---------|
| Safety state | …26a8 | 1 B | SafetyState_t enum |
| Max NTC temp | …26a9 | 4 B | float °C |
| Max current | …26aa | 2 B | uint16 dA |
| Pack voltage setpoint | …26ab | 2 B | uint16 dV (R/W) |
| BMS pack | …26ac | 12 B | pack_mv(4)+current_ma(4)+soc+charge_en+discharge_en+plugged |
| BMS health | …26ad | 23 B | temps+soh+cycles+capacity+runtime |
| BMS cells | …26ae | 44 B | 20×cell_mv + avg_mv + diff_mv |
| GPS | …26af | 26 B | lat+lon+speed+course+heading+range (floats) + fix + sats |

All characteristics: READ + NOTIFY at 1000 ms. Pack voltage: also WRITE.
GPS payload: little-endian IEEE 754 floats.

### GPS Driver (gps_driver.cpp)
- UART2: GPIO 15 (RX) / GPIO 16 (TX), 115200 baud — HGLRC M100 / M100 PRO
- QMC5883L compass: I2C GPIO 8/9 shared bus, address 0x0D
- Range estimation: `remaining_mAh × pack_V / 1000 / GPS_CONSUMPTION_WH_PER_KM`
- Default consumption: 20.0 Wh/km (adjust after real ride data)

### RS485 / JK-BMS (rs485_driver.cpp)
- Main Brain: GPIO 17 (TX) / GPIO 18 (RX), auto-direction module, 115200 8N1
- Temp Node: GPIO 5 (DI) / GPIO 10 (RO) / GPIO 6 (DE/RE), MAX485 5V module, 115200 8N1
- JK-BMS address: per Modbus RTU register map in `JK_BMS/JK_BMS_RS485_Modbus.pdf`

---

## Test Bench Sketch — charger_can_test.ino

Standalone Arduino IDE sketch for functional testing without PlatformIO.
Location: `TC_Charger/test/charger_can_test/charger_can_test.ino`

**Board**: ESP32S3 Dev Module — Flash 16MB, PSRAM OPI, TinyUF2 partition
**CAN pins**: GPIO 8 (CTX) / GPIO 9 (CRX)
**Serial**: 115200, No line ending

| Command | Action |
|---------|--------|
| `m` | Toggle monitor/listen-only mode (no TX, shows all frames raw) |
| `+` | Current +1A (400W limit enforced) |
| `-` | Current −1A |
| `v` | Voltage +0.5V (max 92.4V) |
| `V` | Voltage −0.5V (min 75.6V) |
| `s` | Stop — zero current, ctrl=0x01 |
| `r` | Resume — 2A default, ctrl=0x00 |
| `w` | Send immediate wake-up frame |
| `e` | End charge — ctrl=0x02, charger sleeps |
| `p` | Print current setpoints |

**Defaults**: 84.0V / 2.0A / keep-alive every 3 s
**Safety limits**: 400W max, voltage 75.6–92.4V (84V ±10%)
**Monitor mode**: Restarts TWAI as TWAI_MODE_LISTEN_ONLY — no TX errors, pure sniffer

---

## Charge Profile Knowledge

### CC-CV Charging (Constant Current / Constant Voltage)
Standard lithium charging has two phases:
1. **CC phase** — hold fixed current, voltage rises naturally. This is what the BMS command frame
   controls: set voltage ceiling and current limit, charger operates in CC until pack reaches
   voltage setpoint.
2. **CV phase** — voltage is held at setpoint, current tapers naturally as cells become full.
   Charger transitions automatically when output voltage reaches the commanded ceiling.

### Charge Termination Strategies
- **Fixed voltage cutoff** — stop when pack reaches 84.0V (20 × 4.2V). Simple, slightly
  undercharges top cells due to IR drop.
- **Current taper** — stop when CC-phase current drops below a threshold (e.g. C/20 = 0.3A for
  a 6.6 kW charger). More accurate full charge detection.
- **SOC-based** — use BMS SOC% from RS485 Modbus to stop at e.g. 80% for longevity charging.
- **Timed** — emergency fallback only.

### Derating and Temperature Protection
- Below 0°C: no charging (lithium plating risk)
- 0–5°C: inhibited until pack warms, or use heating mode
- 5–25°C: linear ramp up to full current
- 25–30°C: full current
- 30–55°C: thermal derating per LUT above
- Above 55°C: immediate disable

### Heating Mode
When `BYTE6 = 0x01`, the charger drives the battery heating film directly. The film resistance
must be sized so operating voltage is within battery min/max range. Always send `BYTE5 = 0x01`
(disable) before switching between charging and heating modes.

### Voltage Setpoint Calculation
For balanced full charge: `voltage_dv = num_cells × cell_max_mv / 100`
- 20S, 4.20V/cell: 840 dV = 84.0V
- 20S, 4.15V/cell (longevity): 830 dV = 83.0V
- 20S, 4.10V/cell (storage): 820 dV = 82.0V

### Current Limits
- Charger hardware max: 60A (6.6 kW ÷ 84V ≈ 78A theoretical, limited to 60A by hardware)
- Test bench safe max: 400W ÷ measured_voltage (e.g. 400W ÷ 84V ≈ 4.7A)
- BMS-limited: thermal derating curve above
- PSU-limited during bench test: do not exceed PSU rated output

---

## Troubleshooting Knowledge

### TX ERR 0x103 (ESP_ERR_TIMEOUT = 259)
CAN frame transmitted but no node acknowledged it. CAN requires ACK from at least one other node.
Causes in order of likelihood:
1. Charger not powered / AC not applied
2. CANH/CANL swapped — swap and retry
3. Missing or wrong termination resistor (need 120Ω, 100Ω acceptable for bench test)
4. Charger CAN controller not initialized — some chargers need a few seconds after AC to bring
   CAN online
5. Bus voltage abnormal — measure CANH to CANL: should be ~2.5V idle

### Bus Voltage Readings
| CANH-CANL Reading | Diagnosis |
|------------------|-----------|
| ~2.5V | Healthy idle bus |
| ~1.15V | Charger CAN not active, or wiring issue |
| ~1.43V (resistor removed) | Some signal present but bus not properly terminated or driven |
| 0V | No bus activity at all — check power and wiring |

### Charger Does Not Respond After Wake
- Charger needs AC power applied to wake — CAN alone cannot wake from sleep (0x02 state)
- After AC applied, charger has a boot sequence — CAN may not come online for 2–5 seconds
- Check BYTE6 working_status in RX frame: 0=Undefined means charger still initializing

### No Output From Charger Despite Good CAN
- Check BYTE5 initialization bit — must be 1 before charger outputs
- Check BYTE5 fault bits — any active fault blocks output
- Ensure `BYTE5 = 0x00` in command frame (not 0x01 or 0x02)
- Charger may require valid voltage setpoint — ensure `voltage_dv > 0`

---

## Memory System Integration

This module integrates with the project memory system:
- **Project root CLAUDE.md**: `c:\Scooter_Development\CLAUDE.md` — overall module registry
- **Memory index**: `C:\Users\MSI\.claude\projects\c--Scooter-Development\memory\`
- **Cross-module interfaces**: BMS data consumed from JK_BMS module API; GPS/location from
  GPS driver within this module; mapping/range data can be fed to Mapping_APK module
- **Other modules**: `JK_BMS/` (BMS communication), `Mapping_APK/` (routing), `VESC/` (motor),
  `Scooter_Rebuild/` (hardware)

When another module or agent references charger state, direct them to `can_driver.h` for the
`ChargerStatus_t` and `ChargerCmd_t` structures as the canonical data contract.

---

## Rules for This Agent

1. **Protocol is law** — all CAN frame construction must match the byte layout above exactly.
   Big-endian (Motorola) for all multi-byte fields. Extended 29-bit frame IDs only.
2. **Safety first** — never generate code that commands voltage above 92.4V (20S × 4.62V) or
   current that would exceed 400W during bench testing.
3. **Keep-alive is mandatory** — any code that sends a single command must also establish a
   periodic heartbeat. A single frame without follow-up will cause the charger to disable after 5s.
4. **Mode switching** — always send disable (0x01) before switching BYTE6 mode. Never switch
   between charge and heat mode with output active.
5. **Temperature gates** — never command current > 0 below 0°C or above 55°C. Respect the
   derating LUT for all intermediate temperatures.
6. **Test vs production pins** — test sketch uses GPIO 8/9. Production firmware uses GPIO 4/5.
   Never mix these up in a single build.
7. **Charger sleep** — use 0x01 (disable) for temporary stops. Reserve 0x02 (sleep) for true
   end-of-charge. Sleep requires AC power cycle to recover.
