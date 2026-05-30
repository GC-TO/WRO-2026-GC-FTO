# 🤖 CarloBot — WRO 2026 Future Engineers

<div align="center">

![WRO](https://img.shields.io/badge/WRO-2026-blue?style=for-the-badge)
![Category](https://img.shields.io/badge/Category-Future%20Engineers-orange?style=for-the-badge)
![Team](https://img.shields.io/badge/Team-Future-green?style=for-the-badge)
![Python](https://img.shields.io/badge/MicroPython-ESP32-yellow?style=for-the-badge&logo=python)
![LEGO](https://img.shields.io/badge/LEGO-Inventor%20Hub-red?style=for-the-badge)

</div>

---

## 📋 Table of Contents

- [About the Team](#-about-the-team)
- [About CarloBot](#-about-carlobot)
- [Hardware Architecture](#-hardware-architecture)
- [Software Architecture](#-software-architecture)
- [Repository Structure](#-repository-structure)
- [Challenge Strategies](#-challenge-strategies)
- [How to Run](#-how-to-run)

---

## 👥 About the Team

**Team Name:** Future
**Competition:** WRO 2026 — Future Engineers Category

| Member | Role |
|---|---|
| **Manuel Vásquez** | Programming & Software Development |
| **Antonio Fortín Magaña** | Mechanical Design & Construction |
| **Daniel Salazar** | Electronics (Camera & ESP32), Data Transmission & Documentation |

---

## 🤖 About CarloBot

**CarloBot** is an autonomous self-driving robot built for the WRO 2026 Future Engineers challenge. It combines a LEGO Inventor Hub as the main locomotion controller with an ESP32 microcontroller and an OpenMV camera, creating a distributed sensing and processing architecture capable of navigating complex tracks, avoiding obstacles, and detecting colored traffic signs in real time.

---

## 🔧 Hardware Architecture

CarloBot is built around three processing units that communicate with each other:

```
┌─────────────────┐        UART (115200)       ┌───────────────┐
│   OpenMV Cam    │ ─────────────────────────► │     ESP32     │
│  (Pro_CAM.py)   │   C:<color>,P:<pos>\n       │ (Con_ESP32.py)│
└─────────────────┘                             └───────┬───────┘
                                                        │ PUPRemote (Port D)
                                                        ▼
                                               ┌────────────────┐
                                               │  LEGO Inventor │
                                               │      Hub       │
                                               │ (Ope/Obs_Chall)│
                                               └────────────────┘
```

### Components

| Component | Description |
|---|---|
| **LEGO Inventor Hub** | Main locomotion controller — manages motors, IMU, and driving logic |
| **ESP32 (MicroPython)** | Sensor aggregator — reads 3× TF-Luna LiDAR sensors via I2C and relays OpenMV data to the Hub |
| **OpenMV Camera** | Computer vision module — detects red/green traffic signs and their position (left/right) |
| **TF-Luna LiDAR ×3** | Distance sensors for left, front, and right wall detection (up to ~1000 cm) |
| **Motor E** | Traction motor (Counter-clockwise direction) |
| **Motor B** | Steering motor (Clockwise direction) |

### ESP32 Pin Mapping

| Sensor | SDA | SCL |
|---|---|---|
| TF-Luna 1 (Front) | GPIO 21 | GPIO 22 |
| TF-Luna 2 (Left) | GPIO 13 | GPIO 15 |
| TF-Luna 3 (Right) | GPIO 27 | GPIO 26 |
| UART from OpenMV | RX: GPIO 16 | TX: GPIO 17 |

---

## 💻 Software Architecture

The codebase is split across four Python files, each targeting a specific piece of hardware:

### `Pro_CAM.py` — OpenMV Camera Vision Module
Runs on the **OpenMV Cam**. Captures frames at QQVGA (160×120) resolution and:
- Detects **red** and **green** blobs using LAB color thresholds
- Determines obstacle **position** (left = 1, right = 2) using two ROIs on the lower half of the frame
- Manages **adaptive PWM lighting** (capped at 60%) to maintain stable brightness
- Transmits results over **UART** as `C:<color>,P:<position>\n` at up to 80ms intervals

### `Con_ESP32.py` — ESP32 Sensor Hub
Runs on the **ESP32** in MicroPython. Acts as a sensor bridge:
- Reads distance from **3× TF-Luna LiDAR sensors** over independent SoftI2C buses
- Receives color/position data from the OpenMV camera via **UART**
- Exposes all data to the LEGO Hub via the **PUPRemote** protocol

### `Ope_Chall.py` — Open Challenge Controller
Runs on the **LEGO Inventor Hub**. Controls the robot for the **Open Round**:
- Detects track direction at start (clockwise vs counter-clockwise) using LiDAR
- Implements `sigue_pared1` / `sigue_pared2` for **wall-following** in both directions
- Counts 12 corners to complete 3 full laps, then stops at the finish line

### `Obs_Chall.py` — Obstacle Challenge Controller
Runs on the **LEGO Inventor Hub**. Controls the robot for the **Obstacle Round**:
- Extends the Open Challenge logic with traffic sign awareness from the OpenMV camera
- Adjusts steering dynamically based on detected obstacle color and position

### Shared PID Controllers

Both challenge scripts share two core PID-based movement functions:

**`giro_con_imu(steering, target_angle, speed)`**
Full PID rotation to an exact heading using the IMU.
- Kp = 2.0, Kd = 0.8, Ki = 0 (tuned for sharp, stable turns)
- Tolerance: ±2°, with anti-windup and deadband protection

**`drive_with_heading_lock(heading, speed, sensor, value)`**
Forward driving with active IMU-based heading correction.
- Kp = 2.2, Ki = 0.1, Kd = 1.5
- Supports 5 stop conditions: front LiDAR, right LiDAR, color sensor, time, and distance threshold

---

## 📁 Repository Structure

```
📦 CarloBot/
├── 📄 README.md
├── 🐍 Pro_CAM.py        # OpenMV camera — color detection & UART transmission
├── 🐍 Con_ESP32.py      # ESP32 — LiDAR reading & PUPRemote bridge
├── 🐍 Ope_Chall.py      # LEGO Hub — Open Challenge (wall following, 3 laps)
└── 🐍 Obs_Chall.py      # LEGO Hub — Obstacle Challenge (traffic sign avoidance)
```

---

## 🏁 Challenge Strategies

### Open Challenge
1. CarloBot starts and reads both lateral LiDAR sensors simultaneously
2. If the **right** sensor loses the wall first → **clockwise** lap (`sigue_pared1`)
3. If the **left** sensor loses the wall first → **counter-clockwise** lap (`sigue_pared2`)
4. At each corner, the robot detects the open space, executes a precise 90° PID turn, and reacquires the wall
5. After **12 corners** (3 full laps), the robot drives to the finish line and stops

### Obstacle Challenge
Same as the Open Challenge, with an additional layer:
- The OpenMV camera continuously streams color (red/green) and position (left/right) data
- The robot adjusts its lateral offset when approaching a traffic sign:
  - 🔴 **Red** → pass on the right side
  - 🟢 **Green** → pass on the left side

---

## ▶️ How to Run

### Requirements

- LEGO SPIKE / Inventor Hub with [Pybricks](https://pybricks.com/) firmware
- ESP32 flashed with [MicroPython](https://micropython.org/)
- OpenMV Cam with [OpenMV IDE](https://openmv.io/pages/download)
- `pupremote` library installed on the LEGO Hub and ESP32

### Steps

1. **Flash `Pro_CAM.py`** onto the OpenMV Cam using OpenMV IDE. It will start automatically on power-up.
2. **Flash `Con_ESP32.py`** onto the ESP32 using Thonny or similar. Run `main.py` or copy as `boot.py`.
3. **Connect the ESP32** to LEGO Hub Port D via PUPRemote cable.
4. **Upload** `Ope_Chall.py` **or** `Obs_Chall.py` to the LEGO Hub via Pybricks.
5. Press the **center button** on the Hub to start the run.

---

<div align="center">

**Team Future — WRO 2026**
*Built with 🧱 LEGO · 🐍 Python · 📷 Computer Vision · 📡 LiDAR*

</div>
