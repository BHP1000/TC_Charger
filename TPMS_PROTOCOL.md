# TPMS BLE Protocol Reference — CozyLife / dohome Sensors

Complete protocol decode for the dohome BT5.0 external valve cap TPMS sensors.
Reverse-engineered from live BLE advertisement captures on 2026-04-06.

---

## Credits & References

This protocol was decoded empirically by capturing raw BLE advertisements from
the actual sensors and correlating byte values against known tire pressure
readings from a physical gauge and the CozyLife companion app.

**Research sources that informed the decode approach:**

| Source | Contribution |
|--------|-------------|
| [ra6070/BLE-TPMS](https://github.com/ra6070/BLE-TPMS) | ZEEPIN 18-byte format documentation, ESP32 reference implementation |
| [andi38/TPMS](https://github.com/andi38/TPMS) | BR/SYTPMS 7-byte format, UUID 0x27A5 identification |
| [matthewgream/BluetoothTPMS](https://github.com/matthewgream/BluetoothTPMS) | General TPMS BLE decode methodology |
| [Arduino Forum - BLE TPMS decoding](https://forum.arduino.cc/t/ble-tpms-sensors-decoding/1038638) | Community protocol analysis |
| [Home Assistant Community - BLE TPMS](https://community.home-assistant.io/t/ble-tire-pressure-monitor/509927) | Integration approach |
| [entttom/ESP32-BLE-Tire-Pressure-with-MQTT](https://github.com/entttom/ESP32-BLE-Tire-Pressure-with-MQTT) | ESP32 MQTT TPMS concept |
| [Bluetooth SIG Assigned Numbers](https://www.bluetooth.com/specifications/assigned-numbers/) | Company ID verification (0x0001) |
| CozyLife app (iOS/Android) | Confirmed pressure/temperature readings for cross-reference |

**Key finding:** The dohome/CozyLife sensors use the **ZEEPIN/TP630 protocol**
(company ID 0x0001, 18-byte manufacturer data) which was previously documented
for ZEEPIN-branded sensors. The format is identical despite the different brand.

---

## Hardware

| Item | Detail |
|------|--------|
| Sensors | dohome BT5.0 external valve cap TPMS |
| Amazon ASIN | B0FT38G74C |
| Companion App | CozyLife (Tuya-based platform by doit.am) |
| Battery | CR2032 coin cell (included, ~1-2 year lifespan) |
| Bluetooth | BLE 5.0 advertisement-only (non-connectable) |
| Sensors per pack | 4 included (use 2 for scooter front/rear) |
| Activation | Screw onto tire valve stem (no button, no pairing) |

---

## Protocol: ZEEPIN / CozyLife (18-byte Manufacturer Data)

### How It Works

The sensors are **broadcast-only BLE devices**. They transmit BLE advertisements
containing pressure, temperature, and battery data in the manufacturer-specific
data field (AD type 0xFF). They never accept connections — any number of
receivers can listen simultaneously without interference.

### Identification — How to Recognize This Sensor

| Check | Value |
|-------|-------|
| BLE Type | Non-connectable advertisement |
| AD Type | 0xFF (Manufacturer Specific Data) |
| Company ID | `00 01` (bytes 0-1, little-endian = 0x0001) |
| Payload size | Exactly **18 bytes** |
| Device name | Empty (no name broadcast) |
| Service UUID | None advertised |
| MAC pattern | `XX:ea:ca:50:39:YY` (same batch shares middle bytes) |

**Filter rule:** Accept manufacturer data where `byte[0]==0x00 && byte[1]==0x01 && size==18`.

### Byte Layout (18 bytes)

```
Byte:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
Field: [CID] [----MAC Address----] [--Pressure--] [--Temp----] [B] [A]
```

| Byte(s) | Field | Type | Unit | Description |
|---------|-------|------|------|-------------|
| 0-1 | Company ID | uint16 LE | - | Always `00 01` (Bluetooth SIG: "Nokia Mobile Phones", commonly reused by Chinese TPMS) |
| 2-7 | Sensor MAC | 6 bytes | - | BLE MAC address of the sensor (matches the advertisement source address) |
| 8-11 | Pressure | uint32 LE | Pascals (Pa) | Tire pressure in Pascals. 0 = no pressure (sensor not on valve stem) |
| 12-15 | Temperature | int32 LE | Celsius x 100 | Temperature in hundredths of a degree Celsius. Signed (can go negative in cold) |
| 16 | Battery | uint8 | Percent (%) | Battery level 0-100. Typical new CR2032 = 95% |
| 17 | Alarm | uint8 | Flag | 0x00 = normal, 0x01 = no pressure / rapid loss alarm |

### How to Decode — Step by Step

**Step 1: Extract the raw bytes**

From a BLE scan, you receive manufacturer data like:
```
00 01 81 EA CA 50 39 17 44 B3 03 00 D4 08 00 00 55 00
```

**Step 2: Verify it's a TPMS sensor**

Check bytes 0-1 are `00 01` and total length is 18 bytes. If not, ignore this device.

**Step 3: Extract the MAC address (bytes 2-7)**

Read bytes 2-7 as a MAC: `81:EA:CA:50:39:17`

This should match the BLE advertisement source address. Use this to identify
which sensor is front vs rear.

**Step 4: Decode pressure (bytes 8-11)**

Read as a 32-bit unsigned integer, **little-endian**:
```
Bytes:  44 B3 03 00
Swap:   00 03 B3 44
Hex:    0x0003B344
Dec:    242500
Unit:   Pascals (Pa)
```

Convert to desired unit:
```
kPa = 242500 / 1000        = 242.50 kPa
PSI = 242500 / 6894.76     = 35.17 PSI
bar = 242500 / 100000      = 2.425 bar
atm = 242500 / 101325      = 2.394 atm
```

**Step 5: Decode temperature (bytes 12-15)**

Read as a 32-bit **signed** integer, **little-endian**:
```
Bytes:  D4 08 00 00
Swap:   00 00 08 D4
Hex:    0x000008D4
Dec:    2260
```

Convert:
```
Celsius    = 2260 / 100.0       = 22.60 C
Fahrenheit = (22.60 * 9/5) + 32 = 72.7 F
```

Note: This field is signed (int32). In freezing conditions the value will be
negative. Example: -500 = -5.00C = 23.0F.

**Step 6: Decode battery (byte 16)**

Read directly as unsigned byte:
```
Byte:  55
Dec:   85
Unit:  85% battery remaining
```

Typical values: 95% (new CR2032), 85% (after weeks), below 20% = replace soon.

**Step 7: Decode alarm (byte 17)**

Read directly as unsigned byte:
```
0x00 = Normal (no alarm)
0x01 = Alarm (rapid pressure loss or no pressure detected)
```

### Decoded Examples

**Example 1 — Front tire, 35.2 PSI, 22.6C, battery 85%:**
```
Raw: 00 01 81 EA CA 50 39 17 44 B3 03 00 D4 08 00 00 55 00

Company ID:  00 01
MAC:         81:EA:CA:50:39:17
Pressure:    44 B3 03 00 (LE) = 242500 Pa = 242.50 kPa = 35.17 PSI
Temperature: D4 08 00 00 (LE) = 2260 / 100 = 22.60 C = 72.7 F
Battery:     55 = 85%
Alarm:       00 = normal
```

**Example 2 — Rear tire, 16.1 PSI, 22.1C, battery 95%:**
```
Raw: 00 01 80 EA CA 50 39 04 6C B0 01 00 A2 08 00 00 5F 00

Company ID:  00 01
MAC:         80:EA:CA:50:39:04
Pressure:    6C B0 01 00 (LE) = 110700 Pa = 110.70 kPa = 16.05 PSI
Temperature: A2 08 00 00 (LE) = 2210 / 100 = 22.10 C = 71.8 F
Battery:     5F = 95%
Alarm:       00 = normal
```

**Example 3 — Sensor not on tire (atmospheric / no pressure):**
```
Raw: 00 01 82 EA CA 50 39 61 00 00 00 00 8C 0A 00 00 5F 00

Pressure:    00 00 00 00 = 0 Pa = 0.00 kPa = 0.00 PSI
Temperature: 8C 0A 00 00 = 2700 / 100 = 27.00 C = 80.6 F
Battery:     5F = 95%
Alarm:       00 = normal (sensor reports 0 PSI when valve pin is not depressed)
```

### Conversion Reference

| From | To PSI | To kPa | To bar |
|------|--------|--------|--------|
| Pascals (raw) | / 6894.76 | / 1000 | / 100000 |
| kPa | x 0.145038 | (native) | / 100 |
| PSI | (native) | x 6.89476 | x 0.0689476 |
| bar | x 14.5038 | x 100 | (native) |

**Temperature:**
```
Celsius    = raw_value / 100.0
Fahrenheit = (raw_value / 100.0) * 9.0 / 5.0 + 32.0
```

---

## Broadcast Behavior (Verified from Log Data)

| State | Broadcast Interval | Duration | Pattern |
|-------|-------------------|----------|---------|
| **Active (stationary)** | Every **10 seconds** | ~2-3 minutes | Consistent 10s, occasionally 20s gap |
| **Deep sleep (stationary)** | No broadcasts | ~**8 minutes** | Sensor goes silent to conserve battery |
| **Active (wheel rotating)** | Every **2-5 seconds** | Continuous while moving | Accelerometer detects rotation |
| **Just installed** | First broadcast within ~10 seconds | Then follows active pattern | Screwing onto valve activates sensor |
| **Sensor removed** | Continues broadcasting cached data | Until battery dies or deep sleep | Reports last known pressure |

**Cycle pattern when parked:** 2-3 minutes of 10s broadcasts, then ~8 minutes
of silence, then repeat. This is a battery conservation strategy.

**Verified from Putty log (2026-04-06):**
- Front sensor: 10.0s average broadcast interval, 470s deep sleep observed
- Rear sensor: 10.0s average broadcast interval, 480s deep sleep observed
- Both sensors sleep independently (not synchronized)
- Temperature remained stable at 72F throughout all readings
- PSI fluctuation: 0.1 PSI jitter at ADC boundary (16.0/16.1 alternating)

---

## Sensor Inventory

| Sticker # | MAC Address | Protocol | Status | Notes |
|-----------|-------------|----------|--------|-------|
| #1 | `80:ea:ca:50:39:04` | ZEEPIN 18-byte | **Working** | Currently on rear tire |
| #2 | Unknown | Unknown | **Dead/defective** | Never broadcast, returned item |
| #3 | `81:ea:ca:50:39:17` | ZEEPIN 18-byte | **Working** | Currently on front tire |
| #4 | `82:ea:ca:50:39:61` | ZEEPIN 18-byte | **Working** | Spare (not installed) |

**MAC pattern:** All sensors from the same batch share `XX:ea:ca:50:39:YY`
where the first and last bytes vary.

**Non-TPMS device in vicinity:** `6c:15:db:82:f5:27` (company ID 0x00C4, 9-byte
payload) was initially mistaken for a TPMS sensor. This is an unrelated BLE
device (possibly a neighbor's IoT device). It does NOT follow the ZEEPIN format.

---

## Firmware Implementation Notes

### Signal Processing

**EMA Smoothing (alpha=0.3):**
The raw sensor data has 0.1 PSI jitter at ADC quantization boundaries (e.g.,
bouncing between 16.0 and 16.1 PSI). An Exponential Moving Average with
alpha=0.3 is applied to both PSI and temperature before display/telemetry.

```
smoothed = smoothed + 0.3 * (raw - smoothed)
```

This eliminates the visual bouncing while remaining responsive to real pressure
changes (tracks a genuine drop within 3-4 readings / 30-40 seconds).

**Slow Leak Detection:**
Baseline PSI is recorded on first reading. Every 30 minutes, the smoothed PSI
is compared to the baseline. If pressure has dropped 3+ PSI, a slow leak alert
is triggered. The baseline resets after each check (rolling 30-minute window).

This avoids false alerts from temperature-related pressure changes (typically
~1 PSI per 10F ambient temperature change).

### Scanning Configuration

```
Scan interval:     Every 10 seconds
Scan duration:     5 seconds per cycle (50% duty cycle)
Scan type:         Passive (no scan requests sent)
Duplicate filter:  Disabled (want repeated advertisements)
Max results:       0 (callback-only, no memory storage)
Stale timeout:     10 minutes (sensors sleep ~8 minutes between bursts)
```

### Alert Thresholds (Adjustable via NVS)

| Alert | Default | Command |
|-------|---------|---------|
| Low pressure | < 30 PSI | `TPMSA,30,20,48` |
| Critical pressure | < 20 PSI | (second value in TPMSA) |
| High pressure | > 48 PSI | (third value in TPMSA) |
| High temperature | > 158F (70C) | `TPMST,158,185` |
| Critical temperature | > 185F (85C) | (second value in TPMST) |
| Alert repeat rate | Every 5 minutes | Hardcoded (TPMS_ALERT_REPEAT_MS) |
| Slow leak threshold | 3 PSI drop in 30 min | Hardcoded (TPMS_LEAK_DROP_PSI) |

---

## Non-TPMS BLE Devices (Common False Positives)

These devices are frequently seen during BLE scans and must be filtered out:

| Company ID (bytes 0-1) | Decimal | Source | How to Identify |
|------------------------|---------|--------|-----------------|
| `4C 00` | 0x004C (76) | **Apple** (iPhone, AirPods, Watch, MacBook) | Most common. Payload 6-15 bytes, varies. |
| `F0 01` | 0x01F0 (496) | **Unknown IoT** | 8-byte payload `F0 01 02 03 04 05 06 00` |
| `02 88` | 0x8802 | **Govee** smart home | Name contains "Govee_", 7-byte payload |
| `C4 00` | 0x00C4 (196) | **Unknown** (LG assigned, likely spoofed) | 9-byte payload, never changes. Not TPMS. |

**Filter rule:** Only process devices where `byte[0]==0x00 && byte[1]==0x01 && size==18`.

---

## ESP32 Integration Quick Reference

```cpp
// Include
#include <NimBLEDevice.h>

// Scan setup
NimBLEScan *scan = NimBLEDevice::getScan();
scan->setScanCallbacks(&callbacks, true);     // true = want duplicates
scan->setActiveScan(false);                   // passive scan
scan->setInterval(100);                       // 100ms interval
scan->setWindow(99);                          // 99ms window (near-continuous)
scan->setMaxResults(0);                       // callback-only
scan->start(5000, false);                     // 5 second scan

// In onResult callback:
std::string mfr = dev->getManufacturerData();
if (mfr.size() == 18 && (uint8_t)mfr[0] == 0x00 && (uint8_t)mfr[1] == 0x01) {
    // ZEEPIN TPMS sensor detected
    std::string mac = dev->getAddress().toString();  // "xx:xx:xx:xx:xx:xx"
    
    uint32_t press_pa = (uint8_t)mfr[8]  | ((uint8_t)mfr[9]  << 8) |
                        ((uint8_t)mfr[10] << 16) | ((uint8_t)mfr[11] << 24);
    int32_t  temp_100 = (uint8_t)mfr[12] | ((uint8_t)mfr[13] << 8) |
                        ((uint8_t)mfr[14] << 16) | ((uint8_t)mfr[15] << 24);
    uint8_t  battery  = (uint8_t)mfr[16];
    uint8_t  alarm    = (uint8_t)mfr[17];
    
    float psi    = press_pa / 6894.76f;
    float temp_c = temp_100 / 100.0f;
    float temp_f = temp_c * 9.0f / 5.0f + 32.0f;
}
```
