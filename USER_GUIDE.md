# TC_Charger User Guide

Complete operating instructions for the TC_Charger Main Brain firmware.

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Serial Commands Reference](#serial-commands-reference)
3. [Charge Profiles](#charge-profiles)
4. [Outlet Presets](#outlet-presets)
5. [Buddy Charge Mode](#buddy-charge-mode)
6. [Battery Maintenance Mode](#battery-maintenance-mode)
7. [Scheduled Charging](#scheduled-charging)
8. [Thermal Protection](#thermal-protection)
9. [Sensor Calibration](#sensor-calibration)
10. [TEST/DEV Mode](#testdev-mode)
11. [Energy Tracking](#energy-tracking)
12. [NVS Backup and Restore](#nvs-backup-and-restore)
13. [Remote Control (BLE and WiFi)](#remote-control-ble-and-wifi)
14. [CAN Inter-Module Communication](#can-inter-module-communication)
15. [OLED Display Pages](#oled-display-pages)
16. [RGB LED Status](#rgb-led-status)
17. [Troubleshooting](#troubleshooting)

---

## Quick Start

1. Wire ESP32-S3 to charger via SN65HVD230 or CJMCU-1051 CAN transceiver (3.3V only)
2. Flash the firmware (see README.md for Arduino IDE settings)
3. Open Putty on COM3 at 115200 baud (Terminal > "Implicit CR in every LF" checked)
4. Plug in charger AC power
5. Type `w` to wake the charger (establishes CAN link)
6. Type `p` to verify charger is responding
7. Type `r` to start charging at 84.0V / 2.0A
8. Type `+` to increase current, `-` to decrease
9. Type `s` to stop when done

---

## Serial Commands Reference

All single-character commands execute immediately (no Enter needed).
Multi-character commands require Enter to execute.

### Charge Control

| Key | Command | Description |
|-----|---------|-------------|
| `r` | Resume | Start charging at current voltage/current setpoints |
| `s` | Stop | Disable output, charger stays awake on CAN. Shows session energy summary |
| `e` | End/Sleep | Charger enters sleep mode. Type `w` then `r` to restart |
| `w` | Wake | Send CAN handshake frame. Use after plugging in AC power |

### Setpoint Adjustment

| Key | Command | Description |
|-----|---------|-------------|
| `+` | Current +1A | Increase charge current by 1.0A (max 60A) |
| `-` | Current -1A | Decrease charge current by 1.0A (min 0A) |
| `v` | Voltage +0.5V | Increase target voltage by 0.5V (max 108V) |
| `V` | Voltage -0.5V | Decrease target voltage by 0.5V (min 20V) |

### Modes

| Key | Command | Description |
|-----|---------|-------------|
| `b` | Buddy Mode | Enter buddy charge mode for external batteries |
| `n` | Normal Mode | Exit buddy mode, restore home settings (84.0V / 2.0A) |
| `m` | Maintain | Toggle battery tender mode (auto top-up on 5% SOC drop) |
| `t` | Schedule | Toggle scheduled charging (charge by target time) |

### Profiles

| Key | Profile | Stop Voltage | Per-Cell |
|-----|---------|-------------|----------|
| `1` | MANUAL | No auto-stop | -- |
| `2` | STORAGE | 76.0V | 3.800V |
| `3` | 80% | 78.4V | 3.920V |
| `4` | 85% | 79.2V | 3.960V |
| `5` | 90% | 80.0V | 4.000V |
| `6` | 100% | 84.0V | 4.200V |

### Information

| Key | Command | Description |
|-----|---------|-------------|
| `p` | Print Status | Full detailed report of all settings and charger state |
| `?` | Help | Show command list in terminal |
| `0` | Reset Energy | Reset session and previous energy counters (lifetime preserved) |

### Multi-Character Commands (type then press Enter)

#### Outlet Presets

| Command | Description |
|---------|-------------|
| `OA` | 110V / 15A outlet (1,650W max) |
| `OB` | 110V / 20A outlet (2,200W max) |
| `OC` | 110V / 30A outlet (3,300W max) |
| `OD` | 220V / 15A outlet (3,300W max) |
| `OE` | 220V / 30A outlet (6,600W max) |
| `OG24` | EV charging station at 24A (set your station's amp rating) |
| `OU220,25` | User defined: 220V, 25A (auto-calculates max watts) |
| `OF` | TEST/DEV mode (all safety disabled) |

#### Thermal Zone Thresholds

| Command | Description |
|---------|-------------|
| `TZ1,140` | Set NORMAL/CAUTION boundary (default 140F) |
| `TZ2,194` | Set CAUTION/DANGER boundary (default 194F) |
| `TZ3,248` | Set DANGER/CRITICAL boundary (default 248F) |

#### Sensor Enable/Disable

| Command | Description |
|---------|-------------|
| `TS1,0` | Disable NTC sensor T1 (use TS1,1 to enable) |
| `TS2,0` | Disable NTC sensor T2 |
| `TS3,0` | Disable NTC sensor T3 |
| `TS4,0` | Disable NTC sensor T4 |
| `TS5,0` | Disable NTC sensor T5 |
| `TSB1,0` | Disable BMS Temp 1 |
| `TSB2,0` | Disable BMS Temp 2 |

#### Sensor Calibration

| Command | Description |
|---------|-------------|
| `TC1,-2.0` | Set NTC T1 calibration offset to -2.0F |
| `TC3,1.5` | Set NTC T3 calibration offset to +1.5F |
| `TCB1,-3.0` | Set BMS Temp 1 calibration offset to -3.0F |
| `TCB2,0` | Clear BMS Temp 2 calibration offset |

#### NVS (Non-Volatile Storage)

| Command | Description |
|---------|-------------|
| `ND` | Dump all saved NVS values with restore commands |
| `NLW,1250.3` | Set lifetime Wh (use after flash erase to restore) |
| `NLA,16.2` | Set lifetime Ah |
| `NE,32.0` | Set energy efficiency in Wh/mile |

---

## Charge Profiles

Profiles set an auto-stop voltage. When the pack voltage reaches the target, charging stops automatically.

**How it works:** You set a profile (keys 1-6), then type `r` to start. The charger runs in CC-CV mode until the pack voltage reaches the profile target, then auto-stops.

**Example:** Set profile to 90%, start charging:
```
5       <-- select 90% profile
r       <-- start charging
```
Charger will auto-stop when pack reaches 80.0V (4.00V/cell x 20S).

Profile 1 (MANUAL) disables auto-stop -- you control everything manually with `s`.

---

## Outlet Presets

Outlet presets limit the charge current to prevent overloading your AC circuit.

**Example:** You're at home on a standard 15A outlet:
```
OA      <-- type then press Enter
r       <-- start charging
++++    <-- try to increase current
```
If you try to exceed what the outlet can deliver, the firmware clamps the current:
```
  ** Clamped to 19.6A by 110V/15A outlet (1650W max)
```

**EV charging station:** Since this project uses a CP dummy resistor (no J1772 negotiation), you must manually set the station's amp rating:
```
OG32    <-- EV station rated at 32A
r       <-- start charging
```

**User defined:** For non-standard outlets:
```
OU220,25   <-- 220V outlet with 25A breaker
```

Outlet presets are saved to NVS and remembered across reboots.

---

## Buddy Charge Mode

Charge another vehicle's battery with your charger.

### Steps

1. Type `s` to stop charging your pack (if active)
2. Type `b` to enter buddy mode
3. Disconnect charger output from your 84V pack
4. Connect charger output to buddy's battery
5. Adjust voltage for their pack: `V` or `v` to decrease/increase (range: 20V - 108V)
6. Adjust current: `+` or `-`
7. Type `r` to start charging
8. Monitor the status line for their battery voltage rising
9. When done: type `s` to stop
10. Type `n` to exit buddy mode and restore your home settings

### What buddy mode does

- Stops charging immediately for safety
- Unlocks the full voltage range (20V - 108V)
- Tracks energy separately (buddy Wh/Ah never count toward your lifetime)
- When you exit with `n`, settings automatically restore to 84.0V / 2.0A
- Buddy energy is reported on exit

### Voltage guide for common packs

| Pack | Cells | Full Charge Voltage |
|------|-------|-------------------|
| 36V | 10S | 42.0V |
| 48V | 13S | 54.6V |
| 52V | 14S | 58.8V |
| 60V | 16S | 67.2V |
| 72V | 20S | 84.0V |
| 84V | 24S | 100.8V |

---

## Battery Maintenance Mode

Leave the charger plugged in indefinitely. It monitors the pack voltage and tops up when SOC drops by 5%.

### How to use

1. Set your desired profile: `4` (85%), `5` (90%), etc.
2. Enable maintain mode: `m`
3. Start charging: `r`
4. Charger charges to profile target, then auto-stops
5. Display shows `MAINTAIN` state
6. Every 30 seconds, firmware checks pack voltage
7. When SOC drops 5% below target, auto-starts a top-up charge
8. Cycle repeats indefinitely

### Disable

Type `m` again to toggle off. The charger will stop after the next complete charge and not auto-restart.

---

## Scheduled Charging

"Have my battery at X% by Y time."

### How to use

1. Set your desired profile: `6` (100% for a full charge)
2. Set the target time via WiFi: `http://192.168.4.1/cmd?c=T0800` (ready by 8:00 AM)
3. Set the current time if no GPS fix: `http://192.168.4.1/cmd?c=S2200` (it's 10:00 PM)
4. Enable schedule: `t`
5. Display shows `SCHEDULED` state
6. Firmware calculates: "Need ~650Wh at ~900W average = ~43 min. Start at 7:17 AM."
7. At 7:17 AM, charging auto-starts
8. By 8:00 AM: charge complete

### Time sources

- **GPS:** Automatic if GPS module has a fix
- **WiFi app:** Send current time via `/cmd?c=S1430` (2:30 PM)
- If no time source is available, schedule mode waits until time becomes valid

### Disable

Type `t` again to toggle off.

---

## Thermal Protection

Battery temperature drives charge rate derating. The charger's internal temperature is displayed separately and does NOT affect charge decisions.

### Temperature Zones

| Zone | Temperature | Action |
|------|------------|--------|
| FREEZE | Below 32F (0C) | No charging allowed |
| NORMAL | 32F - 140F (0C - 60C) | Full charge rate |
| CAUTION | 140F - 194F (60C - 90C) | Linear derate from 100% to 50% |
| DANGER | 194F - 248F (90C - 120C) | Linear derate from 50% to 0% |
| CRITICAL | Above 248F (120C) | Immediate shutdown |

### Temperature sources

- NTC T1-T5 (from Temp Node daughter board over RS485)
- BMS Temp 1 and Temp 2 (from JK-BMS over RS485)
- The **highest** enabled sensor reading drives the derating

### Adjusting thresholds

```
TZ1,150    <-- CAUTION starts at 150F instead of 140F
TZ2,200    <-- DANGER starts at 200F instead of 194F
TZ3,260    <-- CRITICAL starts at 260F instead of 248F
```

Thresholds are saved to NVS.

### Disabling sensors

If a sensor is disconnected or giving bad readings:
```
TS3,0      <-- disable NTC T3
TSB1,0     <-- disable BMS Temp 1
```

If ALL battery temp sensors are disabled, no thermal protection is active (same as TEST/DEV mode).

---

## Sensor Calibration

Each temperature sensor can have a calibration offset applied in Fahrenheit.

### Procedure

1. Place sensor next to a known-accurate thermometer
2. Read the displayed value from `p` status report
3. Calculate offset: `offset = actual_temp - displayed_temp`
4. Apply: `TC1,-2.0` (sensor T1 reads 2F too high)

### Examples

```
TC1,-2.0    <-- T1 reads 2F high, subtract 2
TC3,1.5     <-- T3 reads 1.5F low, add 1.5
TCB1,-3.0   <-- BMS Temp 1 reads 3F high
TCB2,0      <-- Clear BMS Temp 2 offset
```

Offsets are saved to NVS and applied before any display or protection logic.

---

## TEST/DEV Mode

Disables ALL safety features for bench testing and development.

### What gets disabled

- Thermal derating (charge at whatever current you set)
- Profile auto-stop (charger runs until you manually stop)
- Outlet power limits (no current clamping)
- Maintenance mode auto-restart
- Scheduled charging

### How to enable

```
OF         <-- type then press Enter
```

### How to disable

Select any normal outlet preset:
```
OA         <-- back to 110V/15A with normal safety
```

### Warning

In TEST/DEV mode you have raw manual control of a 6.6kW charger connected to lithium batteries. There is no overcurrent protection, no thermal shutdown, and no auto-stop. Monitor your battery carefully. The status line shows `[DEV]` as a reminder.

---

## Energy Tracking

### Three tiers

| Tier | What it tracks | When it resets |
|------|---------------|---------------|
| **Session** | Current charge event (Wh, Ah, duration, miles added) | When a new charge starts after End, or manually with `0` |
| **Previous** | Last completed session | Overwritten when new session starts |
| **Lifetime** | All energy ever charged to your pack | Never (saved to NVS flash, survives reboots) |

Buddy mode has its own separate energy counter that never affects your lifetime.

### Range estimation

Default efficiency: 32 Wh/mile. The `p` status report shows:
```
  Charge rate: +28.1 mi/hr at current power
  +5 mi: 11 min | +10 mi: 21 min | +20 mi: 43 min
```

Adjust efficiency: `NE,28.5` (if your scooter does 28.5 Wh/mile)

### Reset session counters

Type `0` to reset session and previous counters. Lifetime is preserved.

---

## NVS Backup and Restore

NVS (Non-Volatile Storage) saves your settings to flash so they survive reboots. However, if you flash with "Erase All Flash" enabled (needed for partition table changes), NVS is wiped.

### Before erasing flash -- back up

Type `ND` + Enter:
```
========= NVS Stored Values =========
  Lifetime Wh:  1250.3  (restore: NLW,1250.3)
  Lifetime Ah:  16.2    (restore: NLA,16.2)
  Wh/mile:      32.0    (restore: NE,32.0)
  Outlet:       110V/20A (restore: OB)
  Thermal Z1:   140F    (restore: TZ1,140)
  Thermal Z2:   194F    (restore: TZ2,194)
  Thermal Z3:   248F    (restore: TZ3,248)
  NTC T2 cal:   -1.5F   (restore: TC2,-1.5)
=====================================
```

Screenshot or copy this output.

### After flashing -- restore

Type each restore command from your backup, pressing Enter after each:
```
NLW,1250.3
NLA,16.2
NE,32.0
OB
TZ1,140
TZ2,194
TZ3,248
TC2,-1.5
```

### What's saved to NVS

| Setting | NVS Key | Set Command |
|---------|---------|-------------|
| Lifetime Wh | `lifetime_wh` | `NLW,value` |
| Lifetime Ah | `lifetime_ah` | `NLA,value` |
| Wh/mile efficiency | `wh_per_mile` | `NE,value` |
| Outlet preset | `outlet` | `OA` through `OF` |
| User outlet voltage | `outlet_v` | `OU110,20` |
| User outlet amps | `outlet_a` | `OU110,20` |
| CAUTION temp threshold | `tz1` | `TZ1,140` |
| DANGER temp threshold | `tz2` | `TZ2,194` |
| CRITICAL temp threshold | `tz3` | `TZ3,248` |
| NTC T1-T5 cal offsets | `cal1`-`cal5` | `TC1,-2.0` |
| BMS T1 cal offset | `calb1` | `TCB1,value` |
| BMS T2 cal offset | `calb2` | `TCB2,value` |

---

## Remote Control (BLE and WiFi)

The firmware runs both BLE and WiFi simultaneously. An Android app (future, separate project) will use these interfaces.

### BLE (Bluetooth Low Energy)

- Device name: `ScooterCharger`
- Service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

| Characteristic | UUID suffix | Direction | Description |
|----------------|-------------|-----------|-------------|
| Command | `...26b0` | Write | Send command characters (same as serial) |
| Charger telemetry | `...26b1` | Notify (1Hz) | 22-byte binary status packet |
| Pack voltage | `...26ab` | Read/Write | Set target voltage (2 bytes, little-endian dV) |
| Safety state | `...26a8` | Notify | 1 byte SafetyState_t enum |
| Max temp | `...26a9` | Notify | 4 bytes float (max NTC temp in C) |
| Charge current | `...26aa` | Notify | 2 bytes uint16 dA |
| BMS pack | `...26ac` | Notify | 12 bytes pack data |
| BMS health | `...26ad` | Notify | 23 bytes health data |
| BMS cells | `...26ae` | Notify | 44 bytes cell voltages |
| GPS | `...26af` | Notify | 26 bytes GPS data |

### WiFi Access Point

- SSID: `ScooterCharger`
- Password: `charge84v`
- IP: `192.168.4.1`

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | API help text |
| `/status` | GET | Full JSON telemetry |
| `/events` | GET | Server-Sent Events (real-time updates) |
| `/cmd?c=r` | GET | Send single-char command |
| `/cmd?c=OB` | GET | Send multi-char command |
| `/cmd?c=T0800` | GET | Set schedule target time (8:00 AM) |
| `/cmd?c=S1430` | GET | Set clock to 2:30 PM |
| `/cmd?c=ND` | GET | Dump NVS values |

### WiFi JSON /status fields

```json
{
  "mode": "HOME",
  "charging": false,
  "set_voltage": 84.0,
  "set_current": 2.0,
  "smooth_voltage": 77.2,
  "smooth_current": 0.0,
  "power_w": 0,
  "charger_responding": true,
  "charger_temp_f": 71,
  "working_status": 2,
  "faults": 0,
  "session_wh": 0.0,
  "session_ah": 0.00,
  "lifetime_wh": 1250.3,
  "lifetime_ah": 16.2,
  "prev_wh": 245.3,
  "miles_added": 0.0,
  "wh_per_mile": 32.0,
  "profile": "MANUAL",
  "profile_stop_v": 0.0,
  "maintain": false,
  "maintain_waiting": false,
  "schedule": false,
  "schedule_time": "8:00",
  "outlet": "110V/15A",
  "outlet_max_w": 1650,
  "test_dev": false,
  "peak_watts": 930,
  "batt_temp_f": 0.0,
  "thermal_zone": "NONE",
  "thermal_derate": 100,
  "time_valid": false,
  "time": "0:00",
  "state": "READY"
}
```

---

## CAN Inter-Module Communication

Other ESP32 modules on the shared CAN bus can control the charger.

### Frame IDs

| ID | Priority | Direction | Purpose |
|----|----------|-----------|---------|
| `0x00010000` | Highest | Any module -> all | Emergency stop (immediate shutdown) |
| `0x1806E5F4` | High | Main Brain -> Charger | Charge commands |
| `0x18FF50E5` | High | Charger -> Main Brain | Charger status |
| `0x1A000001` | Medium | Any module -> Main Brain | Inter-module commands |

### Inter-module command format (ID 0x1A000001)

| Byte 0 | Command | Additional bytes |
|--------|---------|-----------------|
| `0x00` | Emergency stop | None |
| `0x01` | Start charging | None |
| `0x02` | Stop charging | None |
| `0x03` | Set current | Bytes 1-2: current in dA (big-endian) |
| `0x04` | Set voltage | Bytes 1-2: voltage in dV (big-endian) |
| `0x05` | Set profile | Byte 1: profile enum (0-6) |

Voltage and current values are range-checked against hardware limits before applying.

---

## OLED Display Pages

7 pages on the SH1106 128x64 OLED. NEXT button (GPIO 6) cycles pages. HOLD button (GPIO 7) freezes the current page.

| Page | Title | Content |
|------|-------|---------|
| 1/7 | Charger Output | Pack voltage, set voltage, current, power, charging state, faults |
| 2/7 | Ambient Sensors | Main board and daughter board temp/humidity/pressure (inHg) |
| 3/7 | Battery NTC Temps | T1-T5 NTC thermistor readings (Fahrenheit) |
| 4/7 | Pack Status | BMS voltage, current, SOC, Ah remaining, charge/discharge flags |
| 5/7 | Pack Health | MOS temp, battery temps, SOH, cycle count, runtime |
| 6/7 | 20S Cell Volts | All 20 cell voltages in 5x4 grid with min/max/delta summary |
| 7/7 | GPS & Range | Lat/lon, speed (mph), range (miles), compass heading, satellites |

---

## RGB LED Status

The onboard WS2812B LED (GPIO 48) indicates system state:

| Color | Pattern | State |
|-------|---------|-------|
| White | Slow blink | UNPLUGGED (no CAN response) |
| Blue | Slow blink | READY / MAINTAINING / COMPLETE |
| Green | Solid | CHARGING |
| Yellow | Blink | SCHEDULED (waiting for start time) |
| Red | Fast blink | OVERTEMP or FAULT |
| Cyan | Blink | UNDERTEMP (below freezing) |
| Orange | Blink | BMS FAULT |

---

## Electricity Cost Tracking

Track how much you spend on charging and how much you save at free chargers.

### Set your electricity rate

```
NC,0.12     <-- $0.12 per kWh (typical US home rate)
```

### How costs are tracked

| Category | Description |
|----------|-------------|
| Session cost | Current charge session cost, shown in status line |
| Lifetime cost | Total spent on electricity (NVS, survives reboots) |
| Free energy | Total kWh from free chargers (EV stations default to free) |

EV station outlets (`OG`) default to $0.00/kWh. Home outlets use the rate set with `NC`. The status line shows cost when charging:

```
[STATUS] HOME CHARGING | Set:84.0V/12.0A ON | Chgr:77.5V/12.0A 930W 73F WORKING | 45.2Wh +1.4mi $0.05
```

### Restore after flash erase

```
NC,0.12       <-- rate
NF,42.30      <-- lifetime cost
```

---

## Charge Completion Notifications

When a charge completes (profile target reached or manual stop), the firmware:

1. Sends a BLE notification to the app with full session summary (22 bytes)
2. Sounds the buzzer (3 beeps) if a buzzer is wired to the configured GPIO
3. Prints a summary to serial: `[NOTIFY] Charge complete -- $0.15 this session`

The app receives the session packet and logs it to the charge history database.

To wire a buzzer: set `BUZZER_PIN` in the code to your GPIO number (default -1 = none).

---

## Time-to-Target Estimates

The `p` status report shows estimated minutes to reach each profile target:

```
  ---- Time to Target ----
  STORAGE (76.0V): ~8 min
  80% (78.4V): ~12 min
  85% (79.2V): ~14 min
  90% (80.0V): ~17 min
  100% (84.0V): ~28 min
```

Estimates are based on current charge power and voltage delta. They update in real-time as charging progresses.

---

## Thermal Prediction

During charging, the firmware tracks how fast battery temperature is rising. This lets it predict when thermal derating will start.

The `p` report shows:
```
  Thermal:  ~45 min to CAUTION at current rate (1.2F/min rise)
```

This means "at the current charge rate and temperature rise, you have about 45 minutes before the firmware starts reducing charge current."

The prediction improves over time as the firmware learns the thermal behavior of your specific pack at your current charge rate.

---

## GPS Source Selection

Three GPS modes for range calibration accuracy:

| Command | Source | Description |
|---------|--------|-------------|
| `GA` | Onboard only | HGLRC M100, single-band L1, ~2.5m accuracy |
| `GP` | Phone only | Dual-band L1+L5, ~0.3-1.0m accuracy |
| `GB` | Both (default) | Phone primary, onboard fallback |

The phone sends GPS data to the ESP32 via BLE or WiFi. The app handles all calibration calculations and sends the final Wh/mile result.

---

## Troubleshooting

### Charger not responding (UNPLUGGED state)

1. Verify AC power is applied to charger
2. Check CAN wiring: CANH to CANH, CANL to CANL
3. Measure CANH-to-CANL differential: should be ~0V idle (0.0-0.2V)
4. Verify 120 ohm termination resistor across CANH/CANL
5. Verify SN65HVD230/CJMCU-1051 S pin is tied to GND
6. Verify transceiver VCC is 3.3V (never 5V)
7. Type `w` to send a wake frame

### No serial output

- Check Tools > USB CDC On Boot is set to **Disabled**
- Try pressing RST button on the board
- Verify Putty is on the correct COM port at 115200

### Boot loop with partition error

1. Tools > Flash Size > **16MB (128Mb)**
2. Tools > Partition Scheme > **16M Flash (3MB APP/9.9MB FATFS)**
3. Tools > Erase All Flash Before Sketch Upload > **Enabled**
4. Upload sketch
5. Set Erase back to **Disabled** after successful boot
6. Restore NVS values with `ND` backup commands

### Sketch too big

- Tools > Partition Scheme must be **16M Flash (3MB APP/9.9MB FATFS)**
- The default 1.2MB partition is too small for this firmware

### Bouncing voltage/current readings

The charger uses pulse charging. Readings are smoothed with an EMA filter (alpha=0.1). Initial readings take ~10 seconds to settle after starting a charge.

### Commands not working

- Single-char commands (r, s, +, -, etc.) execute immediately, no Enter needed
- Multi-char commands (OA, TZ1,140, NLW,500, etc.) require Enter to execute
- In Putty: Terminal > Local echo > Force off, Local line editing > Force off
