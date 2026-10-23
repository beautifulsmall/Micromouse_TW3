# Micromouse Project Specification

## 1. Overview

This document describes the physical hardware of the micromouse mobile robot and the maze it navigates. It is the specification from which embedded software will be written to program the onboard microcontrollers, enabling the mouse to autonomously navigate the maze in competition.

---

## 2. Hardware Modules

The mouse comprises three on-board hardware modules:

| # | Module | Hardware | Function |
|---|---|---|---|
| 1 | **Main MCU** | Teensy 4.0 (mouse) | Maze solving, motion control, sensor fusion, logging |
| 2 | **Sensor sub-processor** | Teensy 4.0 (sensor board) | Reads 3× S15366-256 laser triangulation sensors, transmits wall distance data to main MCU over Serial2 |
| 3 | **ESP32 radio** | ESP32 Super Mini (mouse) | Receives log/config data from main MCU via UART (Serial5), transmits over ESP-NOW to an external receiver |

Each module operates independently. Communication interfaces between them are defined in the System Architecture section of this document.

---

## 3. Maze & Competition Rules

### 3.1 Maze Dimensions (IEEE Standard)

| Parameter | Value |
|---|---|
| Grid size | 16 × 16 cells |
| Cell size | 180 mm × 180 mm |
| Wall height | 50 mm |
| Wall thickness | 12 mm |
| Post size | 12 mm × 12 mm |

### 3.2 Start & Goal

- **Start cell**: Bottom-left corner (0, 0), facing north
- **Goal area**: 2 × 2 cell region centered at (7–8, 7–8)
- The mouse must return to start after reaching the goal (for scoring runs)

### 3.3 Run Rules

- Multiple solve attempts are permitted within the time limit
- **Time limit**: 5 minutes total per competition round
- Best single run time is used for final ranking
- No remote control or external communication permitted during a run

---

## 4. System Architecture

### 4.1 Component Communication Diagram

```
                        ┌─────────────────────┐
  S15366-256 ×3   ────▶│  Teensy 4.0              │
  (laser triangulation) │  (Sensor sub-processor)  │
                        └────────┬────────────┘
                                 │ UART (wall data)
                                 ▼
  Encoders (×2) ────────────────▶┐
  ICM-45686 IMU ──── SPI ───────▶│
  FRAM ──────────── SPI ───────▶│  Teensy 4.0      │──── UART ────▶ ESP32 (mouse) ──── ESP-NOW ────▶ (external receiver, off-robot)
  TB6612FNG ◀──── PWM/GPIO ─────│  (Main MCU)      │
                                 └──────────────────┘
                                          │
                              Motors (×2)─┘ (via TB6612FNG)
```

### 4.2 Bus Summary — Micromouse

| Bus | Devices |
|---|---|
| SPI | ICM-45686 IMU, FRAM |
| UART 1 — Serial2 (115200) | Sensor sub-processor Teensy 4.0 → Main MCU (wall distance data) |
| UART 2 — Serial5 (460800) | Main MCU → ESP32 Super Mini mouse-side (log data + config) |
| ESP-NOW | ESP32 mouse-side → external receiver (wireless, off-robot) |
| PWM / GPIO | TB6612FNG motor driver (motor control) |
| Quadrature | Left encoder, right encoder |

---

## 5. Pin Connections (Main MCU — Teensy 4.0)

### 5.1 SPI Bus (ICM-45686 IMU & FRAM)

| Teensy 4.0 Pin | Signal | Connected To |
|---|---|---|
| 13 | SCK | ICM-45686 SCK, FRAM SCK |
| 12 | MISO | ICM-45686 SDO, FRAM SO |
| 11 | MOSI | ICM-45686 SDI, FRAM SI |
| 10 | CS0 (IMU) | ICM-45686 CS |
| 6 | CS1 (FRAM) | FRAM CS |

### 5.2 UART — IR Sub-processor (Teensy 4.0)

Serial2, 115200 baud. Receives 4 distance bytes (right, front-L, front-R, left) + CR terminator.

| Teensy 4.0 Pin | Signal | Connected To |
|---|---|---|
| 8 | TX2 | IR Teensy RX |
| 7 | RX2 | IR Teensy TX |

### 5.3 UART — ESP32 Super Mini (WiFi)

Serial5.

| Teensy 4.0 Pin | Signal | Connected To |
|---|---|---|
| 20 | TX5 | ESP32 RX |
| 21 | RX5 | ESP32 TX |

### 5.4 Motor Driver (TB6612FNG)

| Teensy 4.0 Pin | Signal | Connected To |
|---|---|---|
| 9 | PWMA | TB6612 PWMA (Motor 1) |
| 15 | AIN1 | TB6612 AIN1 (Motor 1) |
| 14 | AIN2 | TB6612 AIN2 (Motor 1) |
| 19 | PWMB | TB6612 PWMB (Motor 2) |
| 17 | BIN1 | TB6612 BIN1 (Motor 2) |
| 18 | BIN2 | TB6612 BIN2 (Motor 2) |
| 16 | STBY | TB6612 STBY |

### 5.5 Quadrature Encoders

Hardware quadrature decoder inputs. Left/right defined as viewed from behind the mouse.

| Teensy 4.0 Pin | Signal | Connected To |
|---|---|---|
| 0 | ENC1_A | Left encoder channel A |
| 1 | ENC1_B | Left encoder channel B |
| 2 | ENC2_A | Right encoder channel A |
| 3 | ENC2_B | Right encoder channel B |

- Encoder disc: 82 spokes → 164 edges per revolution per channel
- Wheel circumference: 67.50 mm

### 5.6 Power & Misc

| Teensy 4.0 Pin | Signal | Notes |
|---|---|---|
| 3.3 V | Logic power | Supplied by Teensy onboard regulator |
| GND | Ground | Common ground with all peripherals |
| 4 | LED1 | Status indicator 1 |
| 5 | LED2 | Status indicator 2 |

---

## 6. Hardware Specification

### 6.1 Microcontroller

| Parameter | Requirement |
|---|---|
| MCU | Teensy 4.0 (NXP iMXRT1062, ARM Cortex-M7) |
| Clock speed | 600 MHz |
| Flash | 2 MB (program) + 8 MB external |
| RAM | 1 MB |
| Interfaces required | SPI, I²C, UART, PWM, ADC |

### 6.2 Non-Volatile Storage

| Parameter | Value |
|---|---|
| Device | FRAM, 512 KB (8-bit × 512K), MB85RS4MTY |
| Interface | SPI |
| Purpose | Log data (local storage), configuration data |

### 6.3 Wireless Link (ESP-NOW)

| Parameter | Value |
|---|---|
| Protocol | ESP-NOW (Espressif peer-to-peer, no router required) |
| On-mouse module | ESP32 Super Mini — connected to main MCU via UART (Serial5) |
| Latency | ~1–2 ms |
| Purpose | Real-time log data streaming and parameter configuration to an external (off-robot) receiver |

### 6.4 Drive System

| Parameter | Requirement |
|---|---|
| Configuration | Differential drive (two independent wheels) |
| Motors | NFP D1618 coreless DC, rated 3 V, operated at 7.4 V (2S LiPo) for higher speed, 18,000 RPM max (unloaded), 2.1 A stall current |
| Motor driver | TB6612FNG dual H-bridge |
| Encoders | Quadrature, 82-spoke disc mounted on wheel, 164 edges/rev per channel, 328 counts/rev (×4 quadrature), 0.2058 mm/count, wheel circumference 67.50 mm |
| Drive type | Direct drive, single-stage gear reduction |
| Gear ratio | 11:36 — 11-tooth pinion on motor, 36-tooth cog on wheel |
| Wheel diameter | 21.5 mm |
| Wheel separation (contact points) | 59 mm |

### 6.5 Sensors

| Sensor | Type | Quantity | Purpose |
|---|---|---|---|
| Wall detection | S15366-256 2D laser spot tracking (triangulation), SPI | 3 | Left, right, and front wall distance measurement; data sent to main MCU over UART |
| IMU | ICM-45686 6-DOF (accel + gyro), SPI | 1 | Heading correction, turn accuracy |
| Encoders | Quadrature | 2 | Odometry |

**Sensor placement**:
- 1 × S15366-256 on the left side — single laser, measures left wall distance
- 1 × S15366-256 on the right side — single laser, measures right wall distance
- 1 × S15366-256 at the front — dual laser, measures front wall distance (two points for alignment)

**Measurement principle**: Each sensor uses laser triangulation — a laser projects a spot onto the wall and the S15366-256 2D position-sensitive detector measures the reflected spot position to calculate distance.

**Sensor sub-processor**: A dedicated Teensy 4.0 reads all three S15366-256 sensors over SPI and transmits processed wall distance data to the main Teensy 4.0 over UART (Serial5 on sub-processor → Serial2 on main MCU, 115200 baud).

**Laser assignment**: 4 lasers total — one for the right sensor, two for the front sensor (one per measurement direction), one for the left sensor. Each laser is strobed for 500 µs, the MST pin is pulsed to trigger the sensor, then the result is read over SPI.

| Laser | Sub-processor Pin | Sensor | Measurement |
|---|---|---|---|
| laser_1 | 22 | Sensor 0 (CS pin 2) | Right wall |
| laser_2 | 3 | Sensor 1 (CS pin 5) | Front-right |
| laser_3 | 4 | Sensor 1 (CS pin 5) | Front-left |
| laser_4 | 19 | Sensor 2 (CS pin 9) | Left wall |

Distance conversion will be performed using calibration look-up tables (per channel) rather than closed-form formulas.

**Sensor sub-processor pin assignments (Teensy 4.0)**

| Pin | Signal | Direction |
|---|---|---|
| 0 | sensor_1_output_enable | Output |
| 1 | MST_1 | Output |
| 2 | CS_1 | Output |
| 3 | laser_2 | Output |
| 4 | laser_3 | Output |
| 5 | CS_2 | Output |
| 6 | output_enable_sensor_2 | Output |
| 7 | MST_2 | Output |
| 8 | MST_3 | Output |
| 9 | CS_3 | Output |
| 14 | TG_RESET | Output |
| 15 | SPI_RESET | Output |
| 19 | laser_4 | Output |
| 20 | UART5_TX | Output |
| 21 | UART5_RX | Input |
| 22 | laser_1 | Output |

### 6.6 Power System

| Parameter | Requirement |
|---|---|
| Battery type | LiPo 2S (7.4 V nominal, 8.4 V fully charged) |
| Capacity | 350 mAh |
| Voltage regulation | 3.3 V and 5 V rails for logic; motor rail direct from battery |
| Power switch | Physical kill switch accessible without disassembly |
| Charging | On-board charging circuit or removable pack |

### 6.7 Mechanical

| Parameter | Requirement |
|---|---|
| Footprint | 90 mm (L) × 70 mm (W) |
| Height | 30 mm |
| Weight | 120 g (including battery) |
| Chassis | Custom PCB — structural chassis and circuit board combined |
| Center of gravity | 8 mm above floor, centered laterally between the two motors |

---

## 7. Revision History


| Version | Date | Notes |
|---|---|---|
| 0.1 | 2026-06-06 | Initial draft |
