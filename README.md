# Robocop 2024 – Rescue Maze

This repository contains the source code and design resources for the **Rescue Maze** robot developed by **The Golden Dragon Team** for the Robocop 2024 competition. As the lead programmer, I was responsible for implementing the robot’s core logic, including pathfinding, sensor integration, and PID-based motor control.

---

## Overview

The goal of this project was to build an autonomous robot capable of navigating a dynamic maze, identifying victims, avoiding traps, and mapping the environment in real-time. The robot was designed in accordance with Robocop Rescue Maze competition standards.

**Key Features:**
- Grid-based maze navigation with decision nodes
- Line and wall following using IR and ultrasonic sensors
- PID control for motor accuracy
- Victim detection using temperature and color sensors
- Room entry and exploration with return logic
- Modular design for fast debugging and tuning

---

## Technical Specifications

**Hardware**
- Microcontroller: Arduino Mega 2560  
- Sensors: IR, Ultrasonic, Temperature (MLX90614), Color (TCS3200)  
- Actuators: 4 geared DC motors with encoders  
- Chassis: Custom 3D-printed + laser-cut body  
- Power: 2S LiPo with voltage regulator

**Software**
- Language: Arduino C++  
- Algorithms: PID control, depth-limited search, tile tracking  
- Environment: Arduino IDE  
- Features:
  - Sensor fusion for wall following
  - Grid mapping (2D array)
  - Debugging via serial monitor

---

## Repository Structure

```
Robocop-2024-Rescue-Maze/
├── Arduino control code/     # Arduino sketches and logic modules
├── CAD designs/              # CAD designs and schematics
├── documentation/            # Rulebook references, logic flow diagrams
├── maze_maps/                # Practice layouts and test mazes
├── media/                    # Photos and competition footage
└── README.md                 # Project overview
```

---

## Achievements

- Completed full autonomous maze traversal in test runs
- Implemented adaptive PID for varied line thickness and lighting
- Integrated victim detection with response signaling
- Designed modular codebase for fast tuning and hardware swaps

---
