# LoRa RSSI Tripwire — Direction-Aware Presence Detection

A system that detects people walking through a corridor and determines their direction of travel using sub-GHz LoRa radio signal attenuation. Two sender nodes transmit beacons on separate frequencies; a central receiver measures RSSI dips caused by body obstruction. A Python-based Gaussian Process model learns the "quiet" RSSI baseline and flags anomalies when someone passes through.

## How It Works

```
     S1 (915 MHz)                                S2 (916 MHz)
        │                                           │
        │◄──── LoRa link 1 ────► RX ◄──── LoRa link 2 ────►│
        │                      (receiver)                    │
        └──────────────── corridor ──────────────────────────┘
                    person walks this way →
```

When someone walks through the corridor, they attenuate the LoRa signal between sender and receiver. The RSSI drops by 3–15 dB depending on body size and proximity. By running two independent links (S1–RX and S2–RX), the system detects *which link dips first*, revealing the direction of travel.

The GP baseline model collects 30 seconds of quiet RSSI data during startup, learns the mean signal level per link, and then continuously monitors for deviations exceeding 3.4 dB.

## Hardware

| Device | Board | Role | Frequency |
|--------|-------|------|-----------|
| Sender 1 | Heltec WiFi Kit 32 V3 | Transmit beacons | 915.0 MHz |
| Sender 2 | Heltec WiFi Kit 32 V3 | Transmit beacons | 916.0 MHz |
| Receiver | Heltec WiFi LoRa 32 V3 | Receive + log RSSI | Hops between both |

All three use the SX1262 radio via the RadioLib library. Total hardware cost is ~$45 for three boards.

## Repository Structure

```
├── Sender-2/
│   ├── src/main.cpp          # Sender firmware (ID = "S2", 916 MHz)
│   └── platformio.ini
├── Tripewire-Receiver/
│   ├── src/
│   │   ├── main.cpp          # Receiver firmware (FDMA frequency hopping)
│   │   └── tripwire_gp.py    # GP baseline detection script
│   ├── tripwire-collector.py # Raw serial data collector (saves .pkl/.csv)
│   ├── data/                 # Collected RSSI datasets
│   └── platformio.ini
└── README.md
```

## Setup

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- Python 3.10+ with a virtual environment
- Three Heltec ESP32 V3 boards connected via USB

### 1. Identify USB Ports

```bash
ls /dev/cu.usbserial-*
```

You should see three ports. Note which board is on which port by unplugging/replugging one at a time.

### 2. Flash Sender 2

```bash
cd Sender-2
pio run -t upload
```

The `platformio.ini` specifies the upload port. Edit it if your port differs:

```ini
upload_port = /dev/cu.usbserial-3
monitor_port = /dev/cu.usbserial-3
```

### 3. Flash Sender 1

Sender 1 uses the same firmware structure as Sender 2 but with `SENDER_ID = "S1"` and `LORA_FREQ = 915.0`. Create a copy of `Sender-2/`, change these two values in `src/main.cpp`, update the port in `platformio.ini`, and flash.

Key differences from Sender 2:

```cpp
#define SENDER_ID  "S1"          // was "S2"
#define LORA_FREQ  915.0         // was 916.0
```

### 4. Flash Receiver

```bash
cd Tripewire-Receiver
pio run -t upload
```

Default port is `/dev/cu.usbserial-0001`. Edit `platformio.ini` if needed.

### 5. Install Python Dependencies

```bash
cd Tripewire-Receiver
python -m venv .venv
source .venv/bin/activate
pip install pyserial pandas numpy
```

## Running

### Option A: Raw Data Collection

Collects RSSI data to `.pkl` and `.csv` files for offline analysis.

```bash
python tripwire-collector.py --port /dev/cu.usbserial-0001 --out data/my_run
```

This saves timestamped rows with columns: `wall_time`, `wall_iso`, `rx_millis`, `sender`, `seq`, `raw_rssi`, `filt_rssi`, `baseline`, `delta`, `snr`.

Press `Ctrl+C` to stop. Data auto-saves every 30 seconds and on exit.

### Option B: Live GP Detection

Runs the Gaussian Process anomaly detector in real time.

```bash
cd Tripewire-Receiver/src
python tripwire_gp.py --port /dev/cu.usbserial-0001
```

**Phase 1 — Calibration (first 30 seconds):** Keep the corridor completely clear. The GP learns the baseline RSSI mean and standard deviation per link.

**Phase 2 — Detection:** Every incoming RSSI sample is compared against the learned baseline. If the residual exceeds 3.4 dB, it flags `*** PERSON DETECTED`.

### Option C: Offline GP on Collected Data

```bash
python tripwire_gp.py --pkl ../data/tripwire_run_20260426_144757.pkl
```

## Modifying the Code

### Changing LoRa Parameters

All three devices must use matching parameters (except frequency, which is intentionally different for FDMA). Key defines in each `main.cpp`:

```cpp
#define LORA_BW            125.0   // Bandwidth in kHz
#define LORA_SF            7       // Spreading factor (7-12)
#define LORA_CR            5       // Coding rate (5-8)
#define LORA_SYNC_WORD     0x12    // Private network sync word
#define LORA_POWER         2       // TX power in dBm (2-22)
#define LORA_PREAMBLE      8       // Preamble length
```

**Spreading factor** trades range for airtime. SF7 gives ~50ms airtime per packet, SF9 gives ~165ms. At SF9 with two senders, collisions become nearly guaranteed — SF7 is recommended for dual-sender operation.

**TX power** affects detection sensitivity. Lower power (2 dBm) makes body attenuation more pronounced relative to the signal, improving detection at short range. Higher power (22 dBm) extends range but reduces relative dip depth.

### Adjusting Detection Sensitivity

In `tripwire_gp.py`:

```python
THRESHOLD = 3.4        # dB — lower = more sensitive, higher = fewer false positives
CALIBRATION_SECS = 30  # seconds of quiet data to collect
```

A threshold of 3.4 dB works well for corridor distances of 2–5 meters with TX power at 2 dBm. If you increase TX power or distance, you may need to raise this. If detection is missing smaller people, lower it toward 2.0.

### Changing Sender TX Interval

In each sender's `main.cpp`:

```cpp
#define TX_PERIOD_MS       250     // How often to transmit (milliseconds)
#define TX_JITTER_MS       50      // Random jitter to avoid phase-lock
```

Faster intervals give more samples per crossing event but increase collision probability. At SF7 with 250ms period, each sender uses ~20% duty cycle; two senders at ~40% total is fine.

### Receiver Frequency Hopping

The receiver alternates between S1's and S2's frequencies:

```cpp
#define LORA_FREQ_S1       915.0   // Must match Sender 1
#define LORA_FREQ_S2       916.0   // Must match Sender 2
#define HOP_INTERVAL_MS    150     // Time on each frequency before hopping
```

The hop interval controls how long the receiver listens on each frequency. At 150ms with 250ms TX period, you catch roughly 50% of packets per sender, which is sufficient for detection.

### Adding Sender 3+ (Scaling)

To add more senders, assign each a unique frequency (e.g., 917.0 MHz) and sender ID (`"S3"`). In the receiver, add the new frequency to the hopping schedule and increase `NUM_SENDERS`. The receiver firmware already supports arbitrary sender IDs via the `senderIndex()` lookup.

## Receiver Serial Output Format

The receiver outputs structured lines for machine parsing:

```
DATA,<rx_millis>,<sender>,<seq>,<rssi>,<stddev>,<status>
```

| Field | Description |
|-------|-------------|
| `rx_millis` | Receiver's `millis()` timestamp |
| `sender` | `"S1"` or `"S2"` |
| `seq` | Sender's packet sequence number |
| `rssi` | Received signal strength (dBm) |
| `stddev` | Rolling standard deviation of RSSI window |
| `status` | `CLEAR`, `ALERT`, or `MOTION` |

Lines starting with `#` are debug comments (packet loss reports, etc.) and are ignored by the Python scripts.

## Physical Setup Tips

- Place senders at **torso height** (~1.2m) for maximum body cross-section
- Corridor width of **1–3 meters** works best — wider corridors dilute the signal attenuation
- Keep the receiver roughly **equidistant** between both senders for balanced RSSI
- Avoid placing nodes near metal surfaces or large furniture that cause multipath reflections
- During calibration (first 30s), ensure **no one** is in or near the corridor

## Troubleshooting

**Only seeing one sender:** Check that both senders are flashed with different `SENDER_ID` values and different frequencies. Verify with `pio device monitor` on each sender's port — you should see `tx S1 #N ok` and `tx S2 #N ok`.

**High packet loss:** Reduce TX power from 22 to 2 dBm if nodes are close together. High power at close range can saturate the receiver. Also check that SF matches across all three devices.

**GP always detects anomaly:** The calibration period had interference. Restart the script with the corridor clear and no one nearby for the full 30 seconds.

**GP never detects:** Raise TX power or lower the threshold. If RSSI only fluctuates by ±1 dB when someone walks through, your nodes are too far apart or not at body height.

**Serial read errors on macOS:** The CP2102 USB driver occasionally hiccups. The Python scripts handle this automatically with silent retries. If persistent, try a different USB port or hub.

## Future Work

- **Gaussian Process Regression** for person counting: GP learns RSSI attenuation profiles for 0, 1, 2, 3+ people
- **Softmax classification head** over GP posterior mean and variance for probabilistic count estimation
- **Adaptive Prediction Sets (APS)** for conformal coverage guarantees on person counts
- **Additional sender nodes** for finer spatial resolution and higher count accuracy
- **On-device inference** via quantized models running directly on the ESP32
