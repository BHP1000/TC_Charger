# TC_Charger Communication Interface Blueprint

Complete reference for all communication interfaces supported by the TC_Charger
ESP32-S3 firmware. This document defines how to send commands, receive telemetry,
and integrate with the charger from any external system.

---

## 1. Supported Communication Interfaces

The firmware exposes **four independent communication interfaces**. All four
accept the same command set and return the same data. A command sent via any
interface produces the same result.

| # | Interface | Transport | Port / Address | Use Case |
|---|-----------|-----------|----------------|----------|
| 1 | **Serial** | UART0 (USB) | COM3 @ 115200 baud | Putty terminal, first-time setup, debugging, direct wired control |
| 2 | **BLE** | Bluetooth Low Energy | Device: "ScooterCharger" | Android Charger App, close-range wireless control (~30m range) |
| 3 | **WiFi AP** | HTTP (ESP32 hotspot) | SSID: ScooterCharger / 192.168.4.1 | Direct connection, extended range (~100m open field), always available |
| 4 | **MQTT** | WiFi STA to home network | Configurable broker IP + port | Home Assistant automations, dashboards, remote control while at home |

### 1.1 Simultaneous Operation (All Interfaces Active at Once)

**All interfaces run concurrently.** The ESP32-S3 firmware keeps Serial, BLE,
WiFi AP, and WiFi STA (MQTT) active at all times. Commands received on any
interface are processed immediately and identically. There is no need to
disconnect one interface before using another.

The ESP32-S3 has a single 2.4GHz radio shared between WiFi and BLE. The ESP-IDF
framework handles coexistence internally using time-division multiplexing. Both
WiFi and BLE operate simultaneously with no user intervention. Under heavy
simultaneous use, BLE notification throughput may decrease slightly, but the
charger's 1Hz update rate is well within the coexistence budget.

**WiFi dual mode:** The ESP32 runs WiFi in AP+STA mode simultaneously. The
ScooterCharger hotspot (AP) is always available for direct connection, even when
the ESP32 is also connected to a home WiFi network (STA). When the STA
connection is lost (e.g., driving away from home), the AP remains active.

### 1.2 Real-World Scenario: Garage to Park and Back

| Location | What Happens | Active Interfaces |
|----------|-------------|-------------------|
| **Home garage, plugged in** | ESP32 connected to home WiFi, MQTT publishing to Home Assistant. BLE advertising. AP hotspot running. Serial available via USB. | Serial + BLE + WiFi AP + MQTT |
| **You walk outside with phone** | Charger App connects via BLE. HA still receiving MQTT data. Both connections valid and processing commands. | Serial + BLE + WiFi AP + MQTT |
| **You drive away from home** | WiFi STA loses home router signal. MQTT disconnects automatically. BLE stays connected to your phone. AP hotspot still running. | Serial + BLE + WiFi AP |
| **At the park, BLE signal weak** | Phone disconnects BLE, connects to ScooterCharger WiFi AP hotspot instead. App switches to HTTP/WiFi control. Better range than BLE (~100m vs ~30m in open field). | Serial + WiFi AP |
| **Driving back home** | Phone reconnects BLE as you approach the scooter. When in home WiFi range, ESP32 auto-reconnects STA, MQTT resumes, HA picks up telemetry again. | Serial + BLE + WiFi AP + MQTT |

### 1.3 Interface Selection Guide (for App Developers)

The **firmware** always listens on all interfaces. The **app** (or HA, or user)
decides which interface to use:

- **App logic:** Prefer BLE when connected and signal is strong. Fall back to
  WiFi AP when BLE is unavailable or signal is weak. MQTT is for Home Assistant
  automation, not typically used by the app directly.
- **Home Assistant:** Uses MQTT exclusively. No BLE or WiFi AP interaction.
- **Serial (Putty):** Used for first-time WiFi/MQTT credential setup and
  real-time debugging. Always available when USB cable is connected.
- **Other CAN modules (VESC, BMS):** Use the CAN bus inter-module frames
  (Section 9), not any of the wireless interfaces.

---

## 2. MQTT Configuration

### 2.1 First-Time Setup (via Serial / Putty)

Connect to COM3 at 115200 baud, then type:

```
WS,YourHomeSSID
WP,YourWiFiPassword
WH,192.168.1.100
WM,1883
WC
```

| Command | Description |
|---------|-------------|
| `WS,{ssid}` | Set home WiFi network name |
| `WP,{password}` | Set home WiFi password |
| `WH,{host}` | Set MQTT broker IP or hostname |
| `WM,{port}` | Set MQTT broker port (default: 1883) |
| `WC` | Force connect now |
| `WD` | Disconnect WiFi STA |
| `WI` | Show current WiFi/MQTT status |

All credentials are stored in NVS flash and persist across reboots.

### 2.2 MQTT Topics

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `scooter_charger/state` | ESP32 -> Broker | JSON telemetry, published every 5 seconds |
| `scooter_charger/cmd` | Broker -> ESP32 | Commands (subscribe) |
| `homeassistant/sensor/scooter_charger/*/config` | ESP32 -> Broker | HA auto-discovery (retained, sent once on first connect) |

### 2.3 Home Assistant Auto-Discovery

On first MQTT connect, the firmware publishes retained config messages for these entities:

| Entity ID | Name | Unit | Device Class |
|-----------|------|------|-------------|
| `scooter_charger_voltage` | Charger Voltage | V | voltage |
| `scooter_charger_current` | Charger Current | A | current |
| `scooter_charger_power` | Charger Power | W | power |
| `scooter_charger_session_wh` | Session Energy | Wh | energy |
| `scooter_charger_lifetime_wh` | Lifetime Energy | Wh | energy |
| `scooter_charger_batt_temp` | Battery Temp | F | temperature |
| `scooter_charger_cost_session` | Session Cost | $ | monetary |
| `scooter_charger_state` | Charger State | - | - |

All entities appear under device **"Scooter Charger"** (manufacturer: TC_Charger,
model: HK-LF-115-58 6.6kW).

---

## 3. Telemetry JSON Schema

Published to `scooter_charger/state` every 5 seconds. Also available via
`GET http://192.168.4.1/status` on the WiFi AP.

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
  "working_status": 3,
  "faults": 0,
  "session_wh": 0.0,
  "session_ah": 0.00,
  "lifetime_wh": 1234.5,
  "lifetime_ah": 15.2,
  "prev_wh": 450.0,
  "miles_added": 0.0,
  "wh_per_mile": 32.0,
  "profile": "STORAGE",
  "profile_stop_v": 76.0,
  "maintain": false,
  "maintain_waiting": false,
  "schedule": false,
  "schedule_time": "8:00",
  "outlet": "110V/15A",
  "outlet_max_w": 1650,
  "test_dev": false,
  "peak_watts": 1200,
  "batt_temp_f": 78.5,
  "thermal_zone": "NORMAL",
  "thermal_derate": 100,
  "time_valid": true,
  "time": "14:30",
  "state": "IDLE",
  "cost_kwh": 0.120,
  "cost_session": 0.00,
  "cost_lifetime": 42.30,
  "cost_free_kwh": 15.2,
  "gps_source": "BOTH",
  "thermal_mins_to_caution": -1,
  "tpms_enabled": true,
  "tpms_front_psi": 35.2,
  "tpms_front_temp_f": 72,
  "tpms_front_batt": 85.0,
  "tpms_front_valid": true,
  "tpms_rear_psi": 16.1,
  "tpms_rear_temp_f": 72,
  "tpms_rear_batt": 95.0,
  "tpms_rear_valid": true,
  "wifi_sta": true,
  "mqtt": true,
  "fw": "2.0.0"
}
```

### 3.1 Key Fields

| Field | Type | Description |
|-------|------|-------------|
| `state` | string | Current FSM state (see section 5) |
| `charging` | bool | True if charger output is active |
| `mode` | string | `HOME` or `BUDDY` |
| `smooth_voltage` | float | EMA-smoothed pack voltage (V) |
| `smooth_current` | float | EMA-smoothed charge current (A) |
| `power_w` | int | Instantaneous power (W) |
| `charger_responding` | bool | True if CAN RX from charger is active |
| `charger_temp_f` | int | Charger internal temperature (F) |
| `working_status` | int | 0=Init, 1=Working, 2=Stopped, 3=Standby |
| `faults` | int | Bitwise OR of all charger fault flags |
| `profile` | string | Active charge profile name |
| `profile_stop_v` | float | Auto-stop voltage for current profile (0 = no auto-stop) |
| `batt_temp_f` | float | Highest enabled battery sensor temp (F) |
| `thermal_zone` | string | NORMAL, CAUTION, DANGER, CRITICAL, or FREEZE |
| `thermal_derate` | float | Current derating percentage (100 = full power) |
| `session_wh` | float | Energy delivered this session (Wh) |
| `miles_added` | float | Estimated range added (miles) |
| `wifi_sta` | bool | Home WiFi connected |
| `mqtt` | bool | MQTT broker connected |

---

## 4. Command Reference

Send any of these as the payload to `scooter_charger/cmd` (MQTT),
write to BLE characteristic `26b0`, send via `GET /cmd?c={command}` (WiFi AP),
or type in Putty (Serial).

### 4.1 Single-Character Commands

| Cmd | Action |
|-----|--------|
| `r` | **Start/resume** charging at current setpoints |
| `s` | **Stop** charging (charger stays awake) |
| `e` | **End** charge + charger sleep (needs AC power cycle to restart) |
| `w` | **Wake** -- send CAN frame to charger after AC plug-in |
| `+` | Current **+1A** |
| `-` | Current **-1A** |
| `v` | Voltage **+0.5V** |
| `V` | Voltage **-0.5V** |
| `m` | Toggle **maintain** mode (battery tender) |
| `t` | Toggle **schedule** mode |
| `b` | Enter **buddy** mode (charge another battery) |
| `n` | Exit buddy mode, restore home settings |
| `1` | Profile: **MANUAL** (no auto-stop) |
| `2` | Profile: **STORAGE** (3.80V/cell = 76.0V) |
| `3` | Profile: **80%** (3.92V/cell = 78.4V) |
| `4` | Profile: **85%** (3.96V/cell = 79.2V) |
| `5` | Profile: **90%** (4.00V/cell = 80.0V) |
| `6` | Profile: **100%** (4.20V/cell = 84.0V) |
| `0` | Reset session energy counters |
| `p` | Print full status report |
| `?` | Print help |

### 4.2 Multi-Character Commands

Type the command then press Enter (Serial), or send as a complete string (MQTT/BLE/WiFi).

**Direct Value Set (for app/HA control):**

| Command | Example | Description |
|---------|---------|-------------|
| `SV,{volts}` | `SV,84.0` | Set exact target voltage |
| `SA,{amps}` | `SA,12.0` | Set exact charge current |
| `SM,{volts},{amps}` | `SM,84.0,12.0` | Set voltage + current together |
| `SP,{1-6}` | `SP,3` | Set profile by number |
| `SO,{letter}` | `SO,A` | Set outlet preset by letter |
| `SR` | `SR` | Request full status dump |
| `S{HHMM}` | `S1430` | Set clock to 14:30 |

**Outlet Presets:**

| Command | Outlet | Max Power |
|---------|--------|-----------|
| `OA` | 110V / 15A (standard US) | 1650W |
| `OB` | 110V / 20A (kitchen/garage) | 2200W |
| `OC` | 110V / 30A (max on 110V) | 3300W |
| `OD` | 220V / 15A (standard 220V) | 3300W |
| `OE` | 220V / 30A (dryer/welder) | 6600W |
| `OF` | TEST/DEV (all safety off) | Unlimited |
| `OG{amps}` | EV Station @ N amps | 220V x amps |
| `OU{volts},{amps}` | User defined | volts x amps |

**Thermal / Sensor:**

| Command | Example | Description |
|---------|---------|-------------|
| `TZ{1-3},{tempF}` | `TZ1,140` | Set thermal zone threshold (F) |
| `TS{1-5},{0\|1}` | `TS1,0` | Disable/enable NTC sensor |
| `TSB{1-2},{0\|1}` | `TSB1,0` | Disable/enable BMS temp sensor |
| `TC{1-5},{offset}` | `TC1,-2.0` | NTC calibration offset (F) |
| `TCB{1-2},{offset}` | `TCB1,1.5` | BMS temp calibration offset (F) |
| `TPMS1` / `TPMS0` | | Enable/disable TPMS BLE scanning |
| `TPMSD` | `TPMSD` | Discover sensors (scan 30s, log all MACs) |
| `TPMSF,{MAC}` | `TPMSF,81:ea:ca:50:39:17` | Set front sensor MAC address |
| `TPMSR,{MAC}` | `TPMSR,80:ea:ca:50:39:04` | Set rear sensor MAC address |
| `TPMSI` | `TPMSI` | Show TPMS status, readings, and alert thresholds |
| `TPMSA,{low},{crit},{high}` | `TPMSA,30,20,48` | Set pressure alert thresholds (PSI) |
| `TPMST,{high_f},{crit_f}` | `TPMST,158,185` | Set temperature alert thresholds (F) |
| `TPMSX` | `TPMSX` | Toggle broadcast interval debug mode |
| `T{HHMM}` | `T0800` | Set schedule target time |

**NVS (Persistent Storage):**

| Command | Example | Description |
|---------|---------|-------------|
| `NLW,{wh}` | `NLW,500` | Set lifetime Wh (after flash erase) |
| `NLA,{ah}` | `NLA,6.5` | Set lifetime Ah |
| `NE,{wh/mi}` | `NE,32.0` | Set efficiency (Wh/mile) |
| `NC,{$/kwh}` | `NC,0.12` | Set electricity cost rate |
| `NF,{$}` | `NF,42.30` | Set lifetime cost total |
| `ND` | `ND` | Dump all NVS values |

**WiFi / MQTT:**

| Command | Example | Description |
|---------|---------|-------------|
| `WS,{ssid}` | `WS,MyHome` | Set home WiFi SSID |
| `WP,{pass}` | `WP,secret123` | Set home WiFi password |
| `WH,{host}` | `WH,192.168.1.50` | Set MQTT broker IP/hostname |
| `WM,{port}` | `WM,1883` | Set MQTT broker port |
| `WI` | `WI` | Show WiFi/MQTT connection info |
| `WC` | `WC` | Force WiFi STA connect now |
| `WD` | `WD` | Disconnect WiFi STA |

**GPS Source:**

| Command | Description |
|---------|-------------|
| `GA` | GPS: onboard module only |
| `GP` | GPS: phone only (via BLE/WiFi) |
| `GB` | GPS: both (phone primary, onboard fallback) |
| `PG,{lat},{lon},{mph}` | Push phone GPS data |

---

## 5. Safety State Machine

| State | Value | Description |
|-------|-------|-------------|
| `INIT` | 0 | Boot, initializing drivers |
| `UNPLUGGED` | 1 | No CAN response from charger |
| `IDLE` | 2 | Charger connected, output disabled |
| `CHARGING` | 3 | Actively charging |
| `COMPLETE` | 4 | Auto-stopped at profile target |
| `MAINTAINING` | 5 | Battery tender -- watching voltage for top-up |
| `SCHEDULED` | 6 | Waiting for scheduled charge start time |
| `OVERTEMP` | 7 | Battery over temperature threshold |
| `UNDERTEMP` | 8 | Battery below freezing -- charge inhibited |
| `BMS_FAULT` | 9 | BMS reported a fault |
| `COMM_TIMEOUT` | 10 | CAN communication lost |
| `FAULT` | 11 | Critical fault or emergency stop |

### 5.1 State Transitions

```
Power On --> INIT --> UNPLUGGED (no charger CAN)
                  --> IDLE (charger responding on CAN)

IDLE --> CHARGING (user sends 'r' or CAN/MQTT start command)
     --> SCHEDULED (user enables schedule with 't')

CHARGING --> COMPLETE (pack voltage reaches profile target)
         --> FAULT (CRITICAL thermal zone or emergency stop)
         --> IDLE (user sends 's' to stop)

COMPLETE --> MAINTAINING (if maintain mode enabled)
         --> IDLE (user sends 's')
         --> CHARGING (user sends 'r' to resume, or schedule restarts)

MAINTAINING --> CHARGING (voltage dropped enough for top-up)
            --> IDLE (user disables maintain)

SCHEDULED --> CHARGING (calculated start time reached)

Any State --> UNPLUGGED (CAN timeout, charger disconnected)
Any State --> FAULT (emergency stop from CAN bus)
```

---

## 6. Use Case Examples

### 6.1 Daily Garage Charging (via Home Assistant MQTT)

**Scenario:** Plug in every evening, charge to storage. Full charge only before trips.

**Evening automation (HA YAML):**
```yaml
automation:
  - alias: "Scooter - Charge to Storage on Plug-In"
    trigger:
      - platform: state
        entity_id: sensor.scooter_charger_state
        to: "IDLE"
    action:
      - service: mqtt.publish
        data:
          topic: "scooter_charger/cmd"
          payload: "2"
      - delay: "00:00:01"
      - service: mqtt.publish
        data:
          topic: "scooter_charger/cmd"
          payload: "r"
```

**Pre-trip (send from HA dashboard button or automation):**
```yaml
# Charge to 90% by 8:00 AM tomorrow
- service: mqtt.publish
  data:
    topic: "scooter_charger/cmd"
    payload: "5"
- service: mqtt.publish
  data:
    topic: "scooter_charger/cmd"
    payload: "T0800"
- service: mqtt.publish
  data:
    topic: "scooter_charger/cmd"
    payload: "t"
```

**Charge complete notification:**
```yaml
automation:
  - alias: "Scooter - Charge Complete"
    trigger:
      - platform: state
        entity_id: sensor.scooter_charger_state
        to: "COMPLETE"
    action:
      - service: notify.mobile_app
        data:
          title: "Scooter Ready"
          message: >
            Charged to {{ states('sensor.scooter_charger_voltage') }}V.
            Session: {{ states('sensor.scooter_charger_session_wh') }}Wh
            Cost: ${{ states('sensor.scooter_charger_cost_session') }}
```

### 6.2 Monitor from HA Dashboard

**Lovelace card (entities):**
```yaml
type: entities
title: Scooter Charger
entities:
  - entity: sensor.scooter_charger_state
  - entity: sensor.scooter_charger_voltage
  - entity: sensor.scooter_charger_current
  - entity: sensor.scooter_charger_power
  - entity: sensor.scooter_charger_session_wh
  - entity: sensor.scooter_charger_batt_temp
  - entity: sensor.scooter_charger_cost_session
  - entity: sensor.scooter_charger_lifetime_wh
```

### 6.3 Emergency Stop from HA

```yaml
- service: mqtt.publish
  data:
    topic: "scooter_charger/cmd"
    payload: "s"
```

---

## 7. BLE Interface (Android App)

**BLE Stack:** NimBLE-Arduino 2.5.0 (migrated from stock ESP32 BLE in v2.0.0)

The firmware runs as a BLE **server** (for app communication) and simultaneously
as a BLE **observer** (passive scanning for TPMS tire pressure sensors). NimBLE
handles the coexistence internally.

### 7.1 Service & Characteristics

**Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

| Characteristic | UUID suffix | Direction | Size | Content |
|----------------|-------------|-----------|------|---------|
| Command input | `26b0` | Write | variable | UTF-8 command string |
| Charger telemetry | `26b1` | Notify 1Hz | 22B | voltage, current, session, lifetime, power, miles |
| Session end | `26b2` | Notify | 22B | Summary sent when charge completes |
| Safety state | `26a8` | Notify 1Hz | 1B | State enum (section 5) |
| Max NTC temp | `26a9` | Notify 1Hz | 4B | float Celsius LE |
| Charge current limit | `26aa` | Notify 1Hz | 2B | uint16 deciamps LE |
| Pack voltage setpoint | `26ab` | Read/Write | 2B | uint16 decivolts LE |
| BMS pack | `26ac` | Notify 1Hz | 12B | pack_mv, current_ma, soc, flags |
| BMS health | `26ad` | Notify 1Hz | 23B | temps, soh, cycles, capacity |
| BMS cells | `26ae` | Notify 1Hz | 44B | 20 cell voltages + avg + diff |
| GPS | `26af` | Notify 1Hz | 26B | lat, lon, speed, course, heading, range |

### 7.2 Sending Commands via BLE

Write UTF-8 string to characteristic `26b0`:
- Single-char: write `"r"` to start, `"s"` to stop
- Multi-char: write `"SV,84.0"` to set voltage, `"SP,3"` for 80% profile
- The firmware echoes all BLE commands to serial for debugging

### 7.3 MTU

The firmware accepts MTU negotiation up to 256 bytes. The app should request
MTU 256 on connect to receive full-size characteristics (BMS cells = 44 bytes).

---

### 7.4 TPMS BLE Scanning

The firmware simultaneously scans for BLE TPMS tire pressure sensors while
serving as a BLE server. See `TC_Charger/TPMS_PROTOCOL.md` for the complete
sensor protocol decode, byte layout, conversion formulas, and integration guide.

- Sensors: dohome/CozyLife BT5.0 valve cap TPMS (ZEEPIN 18-byte format)
- Scanning: passive, 5 seconds every 10 seconds
- Data: PSI (EMA smoothed), temperature, battery %, alarm flag
- Alerts: configurable thresholds, rate-limited to every 5 minutes
- Slow leak detection: 3 PSI drop over 30 minutes triggers alert
- MAC addresses stored in NVS (persist across reboots and reflashes)

---

## 8. WiFi AP Interface (Direct HTTP)

**SSID:** `ScooterCharger`  |  **Password:** `charge84v`  |  **IP:** `192.168.4.1`

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | API help text |
| `/status` | GET | Full JSON telemetry (same schema as MQTT state) |
| `/cmd?c={command}` | GET | Send any command from section 4 |
| `/events` | GET | Server-Sent Events stream (1 JSON event per request) |

The AP is always available, even when connected to home WiFi (dual AP+STA mode).

---

## 9. CAN Bus Interface

Other modules on the shared CAN bus (VESC, BMS, lighting) can send commands
to the charger via extended 29-bit CAN frames.

### 9.1 Frame IDs

| ID | Direction | Purpose |
|----|-----------|---------|
| `0x1806E5F4` | ESP32 -> Charger | Charge command (voltage, current, control) |
| `0x18FF50E5` | Charger -> ESP32 | Charger status (voltage, current, faults, temp) |
| `0x00010000` | Any -> ESP32 | Emergency stop (highest priority) |
| `0x1A000001` | Module -> ESP32 | Inter-module command |
| `0x1A000002` | ESP32 -> Modules | Status broadcast |
| `0x1C000010` | VESC -> ESP32 | TPMS tire pressure data |

### 9.2 Inter-Module Commands (0x1A000001)

| Byte 0 | Bytes 1-2 | Description |
|--------|-----------|-------------|
| `0x00` | - | Emergency stop |
| `0x01` | - | Start charging |
| `0x02` | - | Stop charging |
| `0x03` | current_da (BE) | Set charge current (deciamps) |
| `0x04` | voltage_dv (BE) | Set charge voltage (decivolts) |
| `0x05` | profile (B1) | Set charge profile (0-5) |

---

## 10. Charge Profiles

| # | Name | Cell Voltage | Pack Voltage (20S) | Use Case |
|---|------|--------------|--------------------|----------|
| 1 | MANUAL | - | No auto-stop | Full user control |
| 2 | STORAGE | 3.800V | 76.0V | Long-term storage, daily parking |
| 3 | 80% | 3.920V | 78.4V | Daily commute, best longevity |
| 4 | 85% | 3.960V | 79.2V | Moderate use |
| 5 | 90% | 4.000V | 80.0V | Longer trips |
| 6 | 100% | 4.200V | 84.0V | Max range, pre-trip |

---

## 11. Thermal Zones (Battery Protection)

| Zone | Default Threshold | Derating | Action |
|------|-------------------|----------|--------|
| FREEZE | < 32F (0C) | 0% | Charge inhibited |
| NORMAL | 32F - 140F | 100% | Full rate |
| CAUTION | 140F - 194F | 100% -> 50% linear | Reduced current |
| DANGER | 194F - 248F | 50% -> 0% linear | Heavy reduction |
| CRITICAL | > 248F | 0% | Immediate shutdown |

Thresholds adjustable via `TZ1`, `TZ2`, `TZ3` commands. Disabled in TEST/DEV mode.

---

## 12. NVS Persistent Settings

All settings survive reboots. Use `ND` to dump current values with restore commands.

| NVS Key | Type | Default | Set Command |
|---------|------|---------|-------------|
| `lifetime_wh` | float | 0 | `NLW,{val}` |
| `lifetime_ah` | float | 0 | `NLA,{val}` |
| `wh_per_mile` | float | 32.0 | `NE,{val}` |
| `outlet` | uint8 | 0 (110V/15A) | `OA`-`OF` |
| `outlet_v` | uint16 | 110 | `OU{v},{a}` |
| `outlet_a` | uint16 | 15 | `OU{v},{a}` / `OG{a}` |
| `tz1` | float | 140.0 | `TZ1,{val}` |
| `tz2` | float | 194.0 | `TZ2,{val}` |
| `tz3` | float | 248.0 | `TZ3,{val}` |
| `cal1`-`cal5` | float | 0 | `TC{n},{offset}` |
| `calb1`, `calb2` | float | 0 | `TCB{n},{offset}` |
| `cost_kwh` | float | 0.12 | `NC,{val}` |
| `cost_life` | float | 0 | `NF,{val}` |
| `cost_free` | float | 0 | (auto-tracked) |
| `wifi_ssid` | string | "" | `WS,{ssid}` |
| `wifi_pass` | string | "" | `WP,{pass}` |
| `mqtt_host` | string | "" | `WH,{host}` |
| `mqtt_port` | uint16 | 1883 | `WM,{port}` |
