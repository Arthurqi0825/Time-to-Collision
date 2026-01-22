#  TTC (Time-to-Collision) System

A comprehensive ROS-based Time-to-Collision (TTC) detection and visualization system for autonomous vehicle safety using CARLA simulator.

## 📋 Overview

This project implements a real-time TTC calculation system with:
- **Dual vehicle simulation** in CARLA (manual and static vehicles)
- **TTC calculation** with Gaussian noise and communication delay modeling
- **Real-time visualization** in RViz with color-coded warnings
- **On-screen HUD** with persistent collision alerts
- **Collision detection** with configurable thresholds

---

## 🚀 Quick Start

### **Single Command to Run Everything:**

```bash
roslaunch ttc carla_simulator.launch
```

**That's it!** This launches:
- ✅ TTC Calculator
- ✅ TTC Visualizer  
- ✅ CARLA Vehicle Simulation (separate terminal)
- ✅ RViz Visualization

### **Before Running:**

Make sure CARLA simulator is running:
```bash
cd ~/Carla-UE/carla
./CarlaUE4.sh
```

Wait for CARLA to fully load (you'll see "Traffic Manager" message).

---

## 🎯 Features

### 🚗 Vehicle Simulation (`ttc.py`)
- Spawns static and manual-controlled vehicles in CARLA
- Interactive pygame-based HUD with TTC display
- Keyboard control (W/A/S/D) for manual vehicle
- Autopilot toggle for both vehicles (P/O keys)
- **Persistent collision warnings** (stays visible for 5 seconds)
- Color-coded TTC alerts: 🟢 Safe → 🟡 Warning → 🔴 Critical → 🟣 Collision

### 🧮 TTC Calculator (`ttc_calculator.cpp`)
- Calculates true and estimated TTC values
- Applies Gaussian noise to simulate sensor uncertainty
- Models realistic **communication delay** (default 100ms)
- Considers actual vehicle **bounding boxes**
- **COLLISION detection** when TTC < 0.05s
- ROS_WARN terminal alerts for collisions

### 🎨 TTC Visualizer (`ttc_visualizer.cpp`)
- Real-time RViz visualization with colored lines
- Text display showing TTC values and errors
- Color-coded based on warning level
- Automatic color transitions

---

## 📦 Installation

### Prerequisites

```bash
# ROS Noetic (Ubuntu 20.04)
sudo apt-get install ros-noetic-desktop-full

# Required ROS packages
sudo apt-get install ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs

# Python dependencies
pip3 install pygame numpy

# CARLA Simulator (0.9.15)
# Download from https://github.com/carla-simulator/carla/releases
```

### Build

```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Copy ttc package here

# Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Make Python script executable
chmod +x ~/catkin_ws/src/ttc/scripts/ttc.py
```

---

## 🎮 Controls (Pygame Window)

| Key | Action |
|-----|--------|
| **W** | Accelerate forward |
| **S** | Brake |
| **A** | Steer left |
| **D** | Steer right |
| **Space** | Hand brake |
| **P** | Toggle autopilot (manual vehicle) |
| **O** | Toggle autopilot (static vehicle) |
| **ESC** | Quit |

---

## 🎨 What You'll See

### HUD Display (Pygame Window)

```
Server: 60 FPS
Client: 60 FPS
ROS: Connected

Manual Control Vehicle:
  Speed: 45 km/h
  Position: (123.4, 567.8, 0.5)

Static Vehicle:
  Position: (234.5, 678.9, 0.5)
  Distance: 45.2 m

TTC Information:
  🚨 COLLISION DETECTED! 🚨 (3.2s)
  Warning: COLLISION
```

### RViz Visualization

- **Colored lines** between vehicles (changes with TTC)
- **Text display** showing TTC values
- **Bounding boxes** for both vehicles
- Follow vehicles by changing Fixed Frame

---

## ⚙️ Configuration

### Launch File Parameters

```bash
# Low noise, precise
roslaunch ttc carla_simulator.launch noise_std:=0.2

# High communication delay (poor network)
roslaunch ttc carla_simulator.launch comm_delay:=0.3

# More conservative thresholds
roslaunch ttc carla_simulator.launch \
    ttc_warning_threshold:=8.0 \
    ttc_critical_threshold:=3.0

# High update rate for logging
roslaunch ttc carla_simulator.launch update_rate:=200.0
```

### Available Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `noise_std` | 0.5 | Position noise (meters) |
| `comm_delay` | 0.1 | Communication delay (seconds) |
| `ttc_warning_threshold` | 5.0 | Warning threshold (seconds) |
| `ttc_critical_threshold` | 1.6 | Critical threshold (seconds) |
| `update_rate` | 200.0 | Update frequency (Hz) |

---

## 📊 ROS Topics

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ttc/estimated` | Float64 | Estimated TTC value |
| `/ttc/warning_level` | String | SAFE/WARNING/CRITICAL/COLLISION |
| `/ttc/visualization` | MarkerArray | RViz visualization |
| `/vehicle/manual/pose` | PoseStamped | Manual vehicle position |
| `/vehicle/static/pose` | PoseStamped | Static vehicle position |

### Monitor Topics

```bash
# Watch TTC updates
rostopic echo /ttc/estimated

# Watch warning level
rostopic echo /ttc/warning_level

# Check all topics
rostopic list | grep ttc
```

---

## 🎨 RViz Setup

### Follow Vehicles

Change **Fixed Frame** in RViz Global Options:
- `map` - World view (default)
- `manual_vehicle` - Follow manual vehicle
- `static_vehicle` - Follow static vehicle  
- `ttc_center` - Follow midpoint

### Add Displays

1. Click "Add" button
2. Select "By topic"
3. Add:
   - `/ttc/visualization` (MarkerArray)
   - `/ttc/text` (Marker)
   - `/vehicle/*/bounding_box` (Markers)

### Color Meaning

| Color | Warning | TTC Range |
|-------|---------|-----------|
| 🟢 Green | SAFE | > 5.0s |
| 🟡 Yellow | WARNING | 1.6-5.0s |
| 🔴 Red | CRITICAL | 0.05-1.6s |
| 🟣 Magenta | COLLISION | < 0.05s |

---

## 🧪 Test Scenarios

### Test 1: Frontal Collision
1. Launch system
2. Press **W** to accelerate toward static vehicle
3. Watch TTC countdown: Green → Yellow → Red → Magenta
4. See persistent COLLISION alert (5 seconds)

### Test 2: Autopilot
1. Press **O** - static vehicle starts moving
2. Press **P** - manual vehicle autopilot
3. Both drive autonomously
4. Observe TTC changes

### Test 3: Communication Delay
```bash
roslaunch ttc carla_simulator.launch comm_delay:=0.5
```
- Notice 0.5s delay before warning appears
- Simulates V2V communication latency

---

## 🐛 Troubleshooting

### Terminal Closes Immediately

**Check CARLA is running:**
```bash
ps aux | grep Carla
```

**Test script manually:**
```bash
python3 ~/catkin_ws/src/ttc/scripts/ttc.py
```

### No TTC Data

**HUD shows "No topic received"**

```bash
# Check calculator is running
rosnode list | grep ttc

# Check topics exist
rostopic list | grep ttc

# Verify data
rostopic echo /ttc/estimated
```

### Build Errors

```bash
# Install TF2 dependencies
sudo apt-get install ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs

# Clean rebuild
cd ~/catkin_ws
catkin_make clean
catkin_make
```

### Can't Connect to CARLA

```bash
# 1. Start CARLA first
cd ~/Carla-UE/carla
./CarlaUE4.sh

# 2. Wait for "Traffic Manager" message

# 3. Launch ROS
roslaunch ttc carla_simulator.launch
```

---

## 📁 Project Structure

```
ttc/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── carla_simulator.launch       # Main launch file
├── scripts/
│   └── ttc.py                       # Vehicle simulation
├── src/
│   ├── ttc_calculator.cpp           # TTC computation
│   └── ttc_visualizer.cpp           # RViz visualization
└── rviz/
    └── ttc.rviz                     # RViz config
```

---

## 📈 Performance Tips

```bash
# For real-time performance (lower CPU)
roslaunch ttc carla_simulator.launch update_rate:=50.0

# For high-precision logging (higher CPU)
roslaunch ttc carla_simulator.launch update_rate:=200.0

# Minimal noise (ideal sensors)
roslaunch ttc carla_simulator.launch noise_std:=0.1

# Realistic GPS noise
roslaunch ttc carla_simulator.launch noise_std:=0.5
```

---

## 🎓 How TTC is Calculated

1. **Relative Velocity**: `v_rel = v_manual - v_static`
2. **Approach Check**: Verify vehicles moving toward each other
3. **Closing Speed**: Project velocity onto connection line
4. **Distance**: Closest distance between bounding boxes
5. **TTC**: `TTC = distance / closing_speed`

**Special Cases:**
- TTC < 0 → Vehicles diverging → "N/A (Not Approaching)"
- TTC < 0.05s → **COLLISION**

---

## 🚀 Quick Reference

```bash
# Start CARLA
cd ~/Carla-UE/carla && ./CarlaUE4.sh

# Launch system
roslaunch ttc carla_simulator.launch

# Monitor TTC
rostopic echo /ttc/estimated

# Custom config
roslaunch ttc carla_simulator.launch \
    noise_std:=0.3 \
    comm_delay:=0.2 \
    update_rate:=100.0
```

**Controls**: W/A/S/D, P (autopilot), ESC (quit)

**Colors**: 🟢 → 🟡 → 🔴 → 🟣 COLLISION

---

## 📝 License

MIT License

---

## 🙏 Credits

- **CARLA Simulator** - Open-source autonomous driving simulator
- **ROS** - Robot Operating System
- **Pygame** - Python game library

---

**Made for autonomous vehicle safety research** 🚗💨