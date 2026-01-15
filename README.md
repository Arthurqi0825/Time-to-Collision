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
   git clone <repository_url> ttc
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

<!-- ## Configuration

Configure the TTC calculator via `config/param.yaml`:

```yaml
ttc_calculator:
  # Vehicle bounding box dimensions (meters)
  front_vehicle_bbox_length: 4.5    # Front vehicle length
  back_vehicle_bbox_length: 4.5     # Back vehicle length
  
  # Sensor calibration translations
  front_translation: [0.0, 0.0, 0.0]
  back_translation: [0.0, 0.0, 0.0]
  
  # Safety thresholds (seconds)
  safety_threshold: 1.5             # Collision warning threshold
```

### ROS Parameters

Set via `rosparam` or in launch files:

- `noise_std` (double, default: 0.5): Standard deviation of Gaussian noise (meters)
- `ttc_warning_threshold` (double, default: 5.0): Warning TTC threshold (seconds)
- `ttc_critical_threshold` (double, default: 2.0): Critical collision threshold (seconds)
- `update_rate` (double, default: 50.0): Update frequency (Hz)

### Topic Configuration

Customize input/output topics via parameters:

- `static_pose_topic`: Static vehicle pose (default: `/vehicle/static/pose`)
- `static_velocity_topic`: Static vehicle velocity (default: `/vehicle/static/velocity`)
- `manual_pose_topic`: Manually controlled vehicle pose (default: `/vehicle/manual/pose`)
- `manual_velocity_topic`: Manually controlled vehicle velocity (default: `/vehicle/manual/velocity`)
- `ttc_true_topic`: Ground truth TTC output (default: `/ttc/true`)
- `ttc_estimated_topic`: Estimated TTC with noise (default: `/ttc/estimated`)
- `ttc_visualization_topic`: Visualization markers (default: `/ttc/visualization`)

## Input Topics

The calculator subscribes to the following message types:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/vehicle/static/pose` | `geometry_msgs/PoseStamped` | Static vehicle position and orientation |
| `/vehicle/static/velocity` | `geometry_msgs/TwistStamped` | Static vehicle linear/angular velocity |
| `/vehicle/static/bounding_box` | `visualization_msgs/Marker` | Static vehicle dimensions |
| `/vehicle/manual/pose` | `geometry_msgs/PoseStamped` | Manual vehicle position and orientation |
| `/vehicle/manual/velocity` | `geometry_msgs/TwistStamped` | Manual vehicle velocity |
| `/vehicle/manual/bounding_box` | `visualization_msgs/Marker` | Manual vehicle dimensions |

## Output Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/ttc/true` | `std_msgs/Float64` | Ground truth time-to-collision (seconds) |
| `/ttc/estimated` | `std_msgs/Float64` | Estimated TTC with sensor noise (seconds) |
| `/ttc/visualization` | `visualization_msgs/MarkerArray` | Visual markers for RViz display |
| `/ttc/text` | `visualization_msgs/Marker` | Text overlay showing TTC values |
| `/vehicle/manual/pose_true` | `geometry_msgs/PoseStamped` | True pose of manual vehicle |
| `/vehicle/manual/pose_estimated` | `geometry_msgs/PoseStamped` | Estimated pose with noise | -->

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
