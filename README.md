# AGV ROS2 Jazzy Project

## Features

- ROS2 Jazzy
- micro-ROS
- ESP32
- ArUco Localization
- MQTT
- HiveMQ
- HTML GUI

---
## Hardware

- ESP32
- Camera
- Motor Driver
- Encoder
- IMU

---
## Requirements

- Ubuntu 24.04
- ROS2 Jazzy

## Installation

```bash
git clone <repository_url>
cd AGV_Project

chmod +x install.sh
chmod +x setup.sh
chmod +x run.sh

./install.sh
./setup.sh
```

## Run

```bash
./run.sh
```

## Project structure

```
firmware/
    ESP32 Arduino code

ros2_ws/
    ROS2 workspace

web_gui/
    MQTT HTML GUI
```
