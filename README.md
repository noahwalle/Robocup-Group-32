# UC RoboCup 2024 - Group 32 Robot Code

This repository contains the embedded C/C++ code and supporting Python utilities developed by Group 32 for the University of Canterbury 2024 RoboCup competition. The robot was designed to autonomously navigate a 2.4 m × 4.9 m arena, collect metal weights, and deposit them in the designated home base while avoiding fake weights and arena obstacles.

## 🛠 Project Structure

- `src/`: Embedded C/C++ source code running on the Teensy microcontroller.
  - `sensors/`: Sensor drivers (TOF, IMU, inductive proximity, etc.)
  - `motors/`: Motor control modules
  - `states/`: Finite-state machine logic for robot behaviour
  - `scheduler/`: Task scheduling system based on rate-monotonic scheduling
- `python/`: Python scripts for testing and GUI (Tkinter-based control panel)
- `README.md`: This file

## 🧠 Features

- **Autonomous Navigation**: Wall-following algorithm for navigation; later development intended for grid-search.
- **Object Detection**: Differentiates between weights and obstacles using dual-layer TOF sensor setup.
- **Weight Classification**: Uses inductive proximity sensor to identify metal weights and ignore fakes.
- **Pickup Mechanism**: Lead screw-driven pusher feeds weights onto conveyor system.
- **Task Scheduling**: Prioritised task scheduler for sensor reading, motor control, and state management.
- **GUI Interface**: Python-based UI for monitoring and manual control during testing.

## ⚙️ Hardware

- Teensy microcontroller with IO expander
- 12V DC geared motors (143 RPM for drive, 200 RPM for pickup)
- 5x Time-of-Flight (TOF) sensors
- Inductive proximity sensor
- IMU
- Colour sensor (not yet integrated)
- Buck-boost power module

## 🧪 Competition Performance

- **Record**: 1 win, 0 draws, 1 tie, 0 scores
- **Best round**: Successfully collected 1 weight
- **Issues observed**:
  - Occasionally drove over weights without collecting
  - Navigation protocol got stuck on obstacle in Round 2
  - Track came off ramp in Round 3

## 📦 Requirements

- PlatformIO for building embedded code
- Python 3.x with Tkinter and `pyserial` for GUI testing tools

## 🚀 Getting Started

To compile and upload firmware:

```bash
cd src/
platformio run --target upload
