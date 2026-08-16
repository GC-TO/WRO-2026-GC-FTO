# WRO 2026 Future Engineers

<div align="center">

![WRO](https://img.shields.io/badge/WRO-2026-blue?style=for-the-badge)
![Category](https://img.shields.io/badge/Category-Future%20Engineers-orange?style=for-the-badge)
![Team](https://img.shields.io/badge/Team-MAD%20Engineering-green?style=for-the-badge)
![Python](https://img.shields.io/badge/MicroPython-ESP32-yellow?style=for-the-badge&logo=python)
![LEGO](https://img.shields.io/badge/LEGO-Inventor%20Hub-red?style=for-the-badge)
![OpenMV](https://img.shields.io/badge/OpenMV-H7%20Plus-purple?style=for-the-badge)
![LiDAR](https://img.shields.io/badge/LiDAR-TF--Luna%20×3-lightgrey?style=for-the-badge)

</div>

---

## 📋 Table of Contents

- [About the Team](#-about-the-team)
- [About CarloBot](#-about-carlobot)
- [Hardware Architecture](#-hardware-architecture)
- [Power Management](#-power-management)
- [Software Architecture](#-software-architecture)
- [Repository Structure](#-repository-structure)
- [Challenge Strategies](#-challenge-strategies)
- [Testing & Improvements](#-testing--improvements)
- [How to Run](#️-how-to-run)

---

## 👥 About the Team

**Team Name:** MAD Engineering
**Academy:** Game Changer Robotics Academy — El Salvador
**Competition:** WRO 2026 — Future Engineers Category
**Coach:** Carlos España

| Member | Role |
|---|---|
| **Manuel Vásquez** | Lead Programmer — central logic, PID controllers, Open & Obstacle Challenge |
| **Antonio Borst-Fortín** | Mechanical Design & Construction — chassis, steering, structural stability |
| **Daniel Salazar** | Electronics, Camera Configuration, Data Transmission & Documentation |

**MAD Engineering** is the name formed by combining the first letter of each member's name: Manuel, Antonio, and Daniel. **CarloBot** is named after our coach Carlos España, who has guided and supported us throughout the entire development process at the Game Changer Robotics Academy.

We are part of the Future Engineers division of Game Changer Robotics, a robotics academy based in El Salvador focused on developing engineering skills through real competition projects. This is our second international season — we previously competed at WRO Singapore 2025, where we represented El Salvador and brought back critical lessons that shaped the 2026 version of CarloBot.
<div align="center">
  <img width="960" height="1280" alt="WhatsApp Image 2026-06-20 at 15 24 34" src="https://github.com/user-attachments/assets/30821f63-e0bf-467d-9a7f-833e46ebfa98" />
</div>

---

## 🤖 About CarloBot

**CarloBot** is a fully autonomous self-driving robot built for the WRO 2026 Future Engineers challenge. It combines a LEGO Inventor Hub as the main locomotion controller with an ESP32 microcontroller and an OpenMV H7 Plus camera, creating a distributed sensing and processing architecture capable of navigating complex tracks, avoiding obstacles, and detecting colored traffic signs in real time — with no human input during the run.

The robot was completely redesigned this season following our participation at WRO Singapore 2025, where we identified key limitations in the previous version. The most significant structural change was reducing the chassis height from 23 cm to 16.5 cm, which lowered the center of gravity and produced a direct, measurable improvement in stability: the robot went from a maximum of 55% speed in corners without losing traction, to running consistently at 75% — a 36% increase in operational speed without compromising control.

Every design decision in CarloBot responds to a specific engineering constraint identified through testing. The move to three independent LiDAR buses came from an I²C address conflict. The switch to a non-blocking state machine in the ESP32 came from PUPRemote connection losses. The choice of LAB over RGB for color detection came from failures under competition venue lighting. Nothing in this build was theoretical — everything was driven by a test, a failure, and a fix.

### Key Design Principles

| Principle | Implementation | Measurable Result |
|---|---|---|
| **Compactness** | Height reduced 28.3% (23 cm → 16.5 cm), motors redistributed toward center | Corner speed increased from 55% to 75% |
| **Rigidity** | Full LEGO brick chassis, no flex in sensor mounting points | Consistent sensor readings at competition speed |
| **Controllability** | Dual PID system — turn PID (IMU) + trajectory PID (heading lock) | Zero wall contacts in Open Challenge testing |
| **Reliability** | Non-blocking ESP32 architecture, timeout fallbacks on all channels | Zero PUPRemote disconnections since implementation |

---

## 🔧 Hardware Architecture

CarloBot is built around three processing units that communicate in real time:

```
┌─────────────────┐        UART (115200 bps)      ┌───────────────────────────┐
│  OpenMV H7 Plus │ ─────────────────────────────► │          ESP32            │
│  (Pro_CAM.py)   │   C:<color>,P:<position>\n     │      (Con_ESP32.py)       │
│                 │   every 80ms / on change        │                           │
│  480 MHz STM32  │                                 │  ┌─────────────────────┐  │
│  32 MB SDRAM    │                                 │  │ TF-Luna ×3 (LiDAR)  │  │
│  QVGA 320×240   │         SoftI2C @ 400 kHz      │  │ SoftI2C #1 GPIO21/22│  │
└─────────────────┘   ◄─────────────────────────── │  │ SoftI2C #2 GPIO26/27│  │
                          3 independent I²C buses   │  │ SoftI2C #3 GPIO15/13│  │
                                                    │  └─────────────────────┘  │
                                                    └─────────────┬─────────────┘
                                                                  │ PUPRemote
                                                         Port F   │ LPF2 protocol
                                                                  ▼
                                                    ┌─────────────────────────┐
                                                    │     LEGO Inventor Hub   │
                                                    │     (Ope/Obs_Chall.py)  │
                                                    │                         │
                                                    │  Motor E → Traction     │
                                                    │  Motor B → Steering     │
                                                    │  IMU 6-axis (heading)   │
                                                    │  7.3V Li-ion battery    │
                                                    └─────────────────────────┘
```

### Components

| Component | Specs | Role |
|---|---|---|
| **LEGO Inventor Hub** | 7.3V Li-ion · 6 ports · 6-axis IMU | Main locomotion controller — motors, turning, wall-following logic |
| **ESP32 WROOM-32** | 240 MHz dual-core · 520 KB SRAM · 3.0–3.6V | Sensor bridge — reads all 3 LiDARs, relays camera data to Hub via PUPRemote |
| **OpenMV H7 Plus** | STM32H7 480 MHz · 32 MB SDRAM · <150 mA | Computer vision — detects colored blocks, determines position, drives LED brightness PID |
| **TF-Luna LiDAR ×3** | 850 nm VCSEL · 0.2–8 m · ±6 cm · 100 Hz default | Distance sensing — corner detection, wall-following, obstacle position confirmation |
| **Motor E (Port E)** | LEGO Large Angular Motor | Traction — rear axle, counterclockwise direction |
| **Motor B (Port B)** | LEGO Large Angular Motor | Steering — front axle, clockwise direction, ±60 range in competition |

### ESP32 Pin Mapping

| Sensor | SDA | SCL | Bus |
|---|---|---|---|
| TF-Luna 1 (Front) | GPIO 22 | GPIO 21 | SoftI2C #1 |
| TF-Luna 2 (Right) | GPIO 27 | GPIO 26 | SoftI2C #2 |
| TF-Luna 3 (Left) | GPIO 13 | GPIO 15 | SoftI2C #3 |
| UART from OpenMV | RX: GPIO 4 | — | UART1 @ 115,200 bps |
| PUPRemote to Hub | Port F cable | — | LPF2 protocol |

> **Why 3 independent I²C buses?** All three TF-Luna sensors share the same I²C hardware address (0x10). Multiplexing or address remapping adds latency and complexity. Three independent SoftI2C buses eliminate address conflicts and allow all three sensors to be read in a non-blocking rotation without interfering with each other.

---

## ⚡ Power Management

| Component | Voltage | Typical Current | Power |
|---|---|---|---|
| LEGO Hub (processing) | 7.3V | ~300 mA | ~2.2 W |
| Drive motor (under load) | 7.3V | 400–800 mA | 2.9–5.8 W |
| Steering motor | 7.3V | 100–300 mA | 0.7–2.2 W |
| ESP32 (240 MHz, active) | 3.3V | ~50 mA | ~0.17 W |
| 3× LiDAR TF-Luna | 5V | ~210 mA avg | ~1.05 W |
| OpenMV H7 Plus | 3.3–5V | <150 mA | ~0.75 W |
| **Total (estimated)** | — | — | **~8–12 W active** |

The LEGO Hub battery (7.3V / ~2,100 mAh / ~15.3 Wh) provides approximately **1.28 hours** of autonomy at full load. Each competition run (~3 minutes) consumes roughly **0.5 Wh — about 3.3% of total capacity**, guaranteeing stable voltage and consistent motor behavior throughout the entire event. A reserve battery strategy is used: the robot always starts competition at full charge.

---

## 💻 Software Architecture

The codebase is split across four Python files, each targeting a specific piece of hardware:

### `Pro_CAM.py` — OpenMV Camera Vision Module

Runs on the **OpenMV H7 Plus**. Captures frames at **QVGA (320×240)** resolution and:

- Detects **red** and **green** blobs using LAB color thresholds:
  - Red: `(0, 58, 14, 127, -128, 127)`
  - Green: `(30, 90, -128, -10, 0, 127)`
- **Why LAB over RGB?** LAB's L (lightness) channel is fully decoupled from chrominance. Color thresholds remain stable even when ambient light changes — critical in venues with different lighting than our training environment.
- Determines obstacle **position** (left = 1, right = 2) using two ROIs on the lower half of the frame:
  - `ROI_LEFT = (20, 170, 140, 16)` / `ROI_RIGHT = (160, 170, 140, 16)`
- Manages **adaptive PWM lighting** (hard-capped at 60%) via a brightness PID loop:
  - `PWM_new = (1 − α) · PWM_curr + α · (PWM_curr + Kp · brightness_error)`
  - α = 0.20 · Kp = 0.6 · Target brightness = 60 · Max PWM = 60%
- Transmits results over **UART** as `C:<color>,P:<position>\n` at up to 80ms intervals; forces a heartbeat send every 250ms regardless of change
- On startup, sends 10 safe-state messages `C:0,P:0` before entering the main loop

**Camera data codes sent to ESP32:**

| Detected Situation | cam Channel Code |
|---|---|
| Nothing detected | 0 |
| Red — left side | 11 |
| Red — right side | 12 |
| Green — left side | 21 |
| Green — right side | 22 |

### `Con_ESP32.py` — ESP32 Sensor Bridge

Runs on the **ESP32** in MicroPython. Acts as a real-time sensor aggregator and communication bridge:

- Reads distance from **3× TF-Luna LiDAR sensors** over three independent SoftI2C buses at 400 kHz
- Each reading uses command `0x5A 0x05 0x00 0x06 0x65`; 9-byte response validated with checksum `CS = (Σ byte[0..7]) & 0xFF == byte[8]`; distance extracted as `d = byte[2] | (byte[3] << 8)`
- Uses a **non-blocking state machine** per sensor (FREE → WAITING → FREE):
  - State FREE: if 6ms elapsed → send command → transition to WAITING
  - State WAITING: if 3ms elapsed → read → validate → advance to next sensor → FREE
  - Effective update rate: **~37 Hz per sensor** · Full 3-sensor cycle: **~27 ms**
- UART parsing uses direct byte arithmetic — `color = byte - 0x30` — with no `decode()` or `split()` to minimize heap pressure
- Exposes data to the LEGO Hub via **PUPRemote** (LPF2 protocol) on Port F:
  - `cam` channel: `int16` — camera detection code
  - `dist` channel: `3× int16` — LiDAR distances in mm (0–32,767)
- Calls `pr.process()` every ~1ms; if connection is lost, automatically calls `conectar_hub()` to re-negotiate and `restaurar_canales()` to republish last known values — zero manual restart required
- `bytearray` buffer for UART reads avoids allocating new heap objects per chunk
- `gc.collect()` runs every 2 seconds to prevent heap fragmentation
- Timeout fallbacks: LiDAR silent >300ms → distance resets to 0; camera silent >250ms → cam resets to 0

### `Ope_Chall.py` — Open Challenge Controller

Runs on the **LEGO Inventor Hub** via PyBricks. Controls the robot for the **Open Round**:

- Auto-detects track direction at startup from lateral LiDAR readings — no manual setup needed
- If right sensor > 1,000 mm first → **clockwise** (`sigue_pared1`); if left → **counter-clockwise** (`sigue_pared2`)
- Wall-following uses `drive_with_heading_lock()` with ±12° heading corrections relative to accumulated base heading `p`
- Corner detection: > 1,000 mm → 500ms confirmation → PID turn → 3 consecutive wall readings < 1,000 mm → resume
- After **12 corners** (3 full laps), drives forward to finish line and stops

### `Obs_Chall.py` — Obstacle Challenge Controller

Runs on the **LEGO Inventor Hub** via PyBricks. Extends the Open Challenge with full bidirectional obstacle evasion:

- Auto-detects track direction at startup (`sentido = 1` CCW / `sentido = 2` CW) — runs the appropriate exit maneuver (`salida_counter()` or `salida_clock()`) then enters a loop of 11 navigation + corner cycles
- Variable `daniel` accumulates ±90° per completed corner as an absolute heading offset, preventing IMU drift across multiple laps
- Variable `probot` tracks post-evasion state (0 = normal, 1/2 = last block color, 3 = fallback search)

**Clockwise evasion:**
- 🔴 **Red block** → `rojo()`: +55° turn right → front wall follow → return to heading
- 🟢 **Green block** → `verde()`: −40° turn left → left wall follow → return to heading

**Counter-clockwise evasion (mirrored):**
- 🔴 **Red block** → `rojo_ccw()`: +40° turn → left wall follow → return
- 🟢 **Green block** → `verde_ccw()`: −55° turn → front wall follow → return

**Utility functions:**
- `look_for_block()`: creeps forward at low speed until the camera detects a block or the front LiDAR reads below the threshold — prevents acting on stale camera data
- `move_for_distance()`: converts target distance in cm to a timed drive call using wheel circumference (17.5 cm)

### Shared PID Controllers

**`giro_con_imu(steering, target_angle, speed)`** — Full PID rotation to exact IMU heading.

| Parameter | Value | Reasoning |
|---|---|---|
| Kp | 2.0 | Main correction force |
| Ki | 0.0 | Disabled — prevents error accumulation in short turns |
| Kd | 0.8 | Dampens oscillation near target |
| Tolerance | ±2° | Stop condition |
| Deadband | ±1° | Prevents micro-oscillation |
| Min speed | 200 (Obs_Chall) / 100 (Ope_Chall) | Higher floor reduces slow final approach time |

**`drive_with_heading_lock(heading, speed, sensor, value)`** — Forward driving with IMU heading correction.

| Parameter | Value | Reasoning |
|---|---|---|
| Kp | 2.2 | Faster response while moving |
| Ki | 0.1 | Corrects slow drift without instability |
| Kd | 1.5 | Strong damping for straight-line stability |
| MAX_STEER | ±50 | Clamps steering correction |
| Deadband | 0.2° | Prevents unnecessary corrections at speed |
| Confirmation | 5 readings | Prevents false-positive stops |

---

## 📁 Repository Structure

```
📦 WRO-2026-GC-FTO/
├── 📄 README.md
├── 📁 Code/
│   └── 📁 Cam Code/
│       ├── 🐍 Pro_CAM.py        # OpenMV H7 Plus — color detection & UART transmission
│       ├── 🐍 Con_ESP32.py      # ESP32 — LiDAR reading & PUPRemote bridge
│       ├── 🐍 Ope_Chall.py      # LEGO Hub — Open Challenge (wall following, 3 laps)
│       └── 🐍 Obs_Chall.py      # LEGO Hub — Obstacle Challenge (traffic sign avoidance)
├── 📁 Electronics/
│   └── 📁 Diagram Cam/          # Wiring diagrams — ESP32, LiDAR, and OpenMV connections
├── 📁 General Photos/
│   └── 📁 DREAM TEAM/
│       └── 📁 At Work/          # Team photos during development and testing sessions
└── 📁 Mechanics/
    └── 📁 3D Models/
        └── 📁 Sensor Case/      # 3D printed cases for LiDAR and camera sensor mounting
```

---

## 🏁 Challenge Strategies

### Open Challenge

1. CarloBot starts and reads both lateral LiDAR sensors simultaneously
2. If the **right** sensor exceeds 1,000 mm first → **clockwise** lap (`sigue_pared1`)
3. If the **left** sensor exceeds 1,000 mm first → **counter-clockwise** lap (`sigue_pared2`)
4. At each corner: detect open space (> 1,000 mm) → 500ms confirmation → 90° IMU-guided turn → wall reacquisition (3 consecutive readings < 1,000 mm) → resume
5. After **12 corners** (3 full laps), robot drives to finish line and stops

**Competition speed:** `drive_speed = 700` — balanced between speed and reliability.

### Obstacle Challenge

Same as Open Challenge, plus:

- 🔴 **Red** → pass right — `rojo()`: +55° turn → wall follow → return
- 🟢 **Green** → pass left — `verde()`: −40° turn → wall follow → return
- Camera re-read after each evasion confirms clearance before resuming

**Competition speed:** `drive_speed = 500` (CW) / `drive_speed = 400` (CCW) — reduced to give camera pipeline sufficient processing time.

### Risk Management

| Risk | Root Cause | Mitigation |
|---|---|---|
| LiDAR dropout | Loose cable or I²C timeout | `DIST_TIMEOUT_MS = 300` → distance resets to 0 |
| Stale camera reading | UART loss or OpenMV lag | `CAM_TIMEOUT_MS = 250` → cam resets to 0 |
| IMU heading drift | Accumulated angle error | `imu.reset_heading(0)` at every program start |
| False corner trigger | Track irregularity or object | 5 consecutive readings confirmation required |
| PUPRemote disconnect | Blocking code in ESP32 | `pr.process()` called every ~1ms, non-blocking architecture |
| Battery performance drop | Discharge during session | Reserve battery strategy; checked via `battery.voltage()` at startup |

---

## 🧪 Testing & Improvements

### Test Log

| # | Objective | Failure Observed | Change Applied | Status |
|---|---|---|---|---|
| 1 | Reduce robot height | High-speed corner instability | Motor redistribution → 16.5 cm | ✅ Fixed |
| 2 | LiDAR corner threshold | False corners at 800 mm | Raised threshold to 1,000 mm | ✅ Fixed |
| 3 | Non-blocking ESP32 | PUPRemote lost connection due to `sleep_ms()` | State machine FREE/WAITING per sensor | ✅ Fixed |
| 4 | Red LAB threshold | False negatives under competition lighting | Broadened LAB range to `(0, 58, 14, 127, -128, 127)` | ✅ Fixed |
| 5 | Post-corner trajectory | Robot loses wall reference after evasion at corner | Under development | ⚠️ Pending |

### Key Engineering Lessons

**Software architecture matters as much as hardware.** The `sleep_ms()` blocking PUPRemote was not apparent until we measured the actual Hub update frequency. Once we understood the root cause, the fix was straightforward — but finding it required systematic measurement.

**LAB over RGB is an engineering decision, not a preference.** RGB thresholds shift with ambient light intensity. LAB's L channel separates luminance from chrominance — color detection stays stable even when total brightness changes. This eliminated an entire category of failures.

**Lower center of mass equals more stability — and the data proves it.** Height reduction from 23 cm to 16.5 cm allowed a speed increase from 55% to 75% in corners. The improvement was immediate and directly tied to the physical change.

**Test one variable at a time.** Changing multiple parameters simultaneously makes it impossible to identify which change caused the improvement — or the failure.

---

## ▶️ How to Run

### Requirements

- LEGO SPIKE / Inventor Hub with [Pybricks](https://pybricks.com/) firmware
- ESP32 flashed with [MicroPython](https://micropython.org/) (v1.20+)
- OpenMV H7 Plus with [OpenMV IDE](https://openmv.io/pages/download)
- `pupremote` library installed on both the LEGO Hub and ESP32

### Steps

1. **Flash `Pro_CAM.py`** onto the OpenMV H7 Plus via OpenMV IDE. Starts automatically on power-up.
2. **Flash `Con_ESP32.py`** onto the ESP32 via Thonny. Copy as `boot.py` for automatic startup.
3. **Connect the ESP32** to LEGO Hub Port F via PUPRemote cable (modified PoweredUp cable with SDA, SCL, 5V, GND exposed).
4. **Upload** `Ope_Chall.py` or `Obs_Chall.py` to the LEGO Hub via Pybricks.
5. Press the **center button** on the Hub to start the run.

> **Startup order matters:** Power the OpenMV camera first, then the ESP32, then press the Hub button. The camera sends 10 safe-state `C:0,P:0` messages on boot — starting in the wrong order can result in stale zero readings on the first few cam values.

---

<div align="center">

**MAD Engineering — WRO 2026 Future Engineers**
*Game Changer Robotics Academy · El Salvador*
*Built with 🧱 LEGO · 🐍 MicroPython · 📷 Computer Vision · 📡 LiDAR · ⚙️ PID Control*

</div>
