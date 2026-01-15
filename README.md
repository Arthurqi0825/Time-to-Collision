# TTC Calculator

A ROS (Robot Operating System) package for calculating **Time-to-Collision (TTC)** between vehicles in autonomous driving scenarios. This package computes collision time estimates with support for noisy sensor data and provides real-time visualization.

## Project Structure

```
ttc/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS package metadata
├── README.md                   # This file
├── config/
│   └── param.yaml              # Configuration parameters
├── include/ttc/                # Header files
├── launch/
│   └── ttc_calculator.launch   # Launch file
├── rviz/
│   └── ttc.rviz                # RViz configuration
├── scripts/                    # Python simulation scripts
│   ├── carla_rospub.py
│   ├── vehicle_position.py
│   ├── manual_control.py
│   ├── two_cars.py
│   └── combine.py
├── src/
│   ├── ttc_calculator.cpp      # Main TTC calculation node
│   ├── ttc_multiple_dirving_direction_in_x_y.cpp  # Alternative implementation
│   └── ttc_calculator copy.cpp # Backup
└── bags/                       # Sample bagfiles for testing
    ├── 2026-01-14-20-59-14.bag
    └── 2026-01-14-21-00-03.bag
```

## Dependencies

- **ROS** (tested with ROS Noetic)
- **C++11** or higher
- **Eigen3** (linear algebra library)
- **geometry_msgs**, **visualization_msgs**, **std_msgs** (ROS message types)

## Installation

1. Clone this package into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Arthurqi0825/Time-to-Collision.git ttc
   ```

2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source the setup file:
   ```bash
   source devel/setup.bash
   ```

## Usage

### Running the TTC Calculator

Start the TTC calculator node:
```bash
roslaunch ttc ttc_calculator.launch
```

Or manually run:
```bash
rosrun ttc ttc_calculator
```

### Python Simulation Scripts

Several Python scripts are provided for testing and simulation:

- **`carla_rospub.py`**: Publishes vehicle data from CARLA simulator
- **`vehicle_position.py`**: Generates vehicle position trajectories
- **`manual_control.py`**: Manual vehicle control interface
- **`two_cars.py`**: Simulation setup for two-vehicle scenarios
- **`combine.py`**: Combines multiple data sources


### Collecting Custom Data

To collect your own vehicle data, use the `combine.py` script:

```bash
python scripts/combine.py
```

This script aggregates data from multiple sources (CARLA simulator, vehicle sensors, or manual inputs) 

```bash
#Original carla manual control scripts 
python scripts/manual_control
``` 
## Visualization

View real-time TTC calculations and collision states in RViz:

```bash
rviz -d src/ttc/rviz/ttc.rviz
```

The RViz configuration displays:
- Vehicle positions and bounding boxes
- TTC warning/critical status indicators
- Collision prediction visualization

## Data Bags

Test data is provided in `bags/` directory:

- `2026-01-14-20-59-14.bag`: Sample driving scenario
- `2026-01-14-21-00-03.bag`: Alternative scenario

Replay bagfiles:
```bash
rosbag play bags/2026-01-14-20-59-14.bag
```


## Contributing

For bug reports and feature requests, please open an issue or submit a pull request.
