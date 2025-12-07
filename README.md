# 🤖 My_Bot - Autonomous Navigation with Adaptive Path Planning

> **An intelligent differential drive robot featuring advanced path smoothing, trajectory generation, and robust tracking control with real-time error recovery.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)](http://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE.md)

---

## 🌟 Overview

**My_Bot** is a comprehensive robotics navigation system that combines cutting-edge path planning algorithms with real-time adaptive control. Built on ROS 2 Humble, this project demonstrates professional-grade autonomous navigation with features like cubic spline path smoothing, curvature-based velocity optimization, and intelligent error recovery.

### Key Highlights

- **🎯 Adaptive Path Planning**: Automatically recalibrates paths when the robot is moved or encounters obstacles
- **📈 Smooth Trajectory Generation**: Cubic spline interpolation for natural, efficient motion
- **⚡ Dynamic Velocity Control**: Curvature-aware speed optimization for safe cornering
- **📊 Comprehensive Logging**: Automatic CSV generation for trajectory analysis and validation
- **🔄 Real-time Error Recovery**: Seamlessly handles disturbances and position changes
- **🎨 5 Pattern Test Suite**: Circle, Square, Zigzag, Figure-Eight, and Spiral patterns for thorough testing

---

## 📦 Package Structure

```
my_bot/
├── config/                         # Configuration and data files
│   ├── trajectory_execution.csv    # Timestamped robot trajectory data
│   ├── waypoints_and_paths.csv     # Waypoint and smooth path logs
│   └── view_bot.rviz               # RViz configuration
├── description/                    # Robot URDF/Xacro files
│   ├── robot.urdf.xacro           # Main robot description
│   ├── lidar.xacro                # LiDAR sensor configuration
│   ├── camera.xacro               # Camera configuration
│   └── gazebo_control.xacro       # Gazebo plugins
├── launch/                         # Launch files
│   ├── launch_sim.launch.py       # Main simulation launcher
│   └── rsp.launch.py              # Robot state publisher
├── scripts/                        # Python nodes
│   ├── waypoint.py                # Waypoint generator (5 patterns)
│   ├── path.py                    # Path smoother (cubic splines)
│   ├── path_executor.py           # Path tracking controller
│   └── tf_test.py                 # TF coordinate monitor
├── worlds/                         # Gazebo world files
│   ├── empty.world                # Empty testing environment
│   └── wall.world                 # Environment with obstacles

```

---

## 🚀 Features

### 1. **Intelligent Path Smoothing**
- Cubic spline interpolation for smooth, continuous paths
- Tangent-based orientation calculation
- Automatic robot position integration
- 500-point path resolution for precision

### 2. **Advanced Velocity Control**
- **Curvature-based speed adaptation**: Automatically slows down for sharp turns
- **Look-ahead turn detection**: Anticipates upcoming curves
- **Smooth acceleration/deceleration**: Respects physical constraints (1.0 m/s²)
- **Velocity smoothing**: Prevents jerky motion with configurable smoothing factor

### 3. **Robust Error Handling**
- **Real-time path recalibration**: Move the robot in Gazebo during execution - it automatically recovers!
- **Pattern change detection**: Distinguishes between path updates and new patterns
- **Velocity maintenance**: Preserves momentum across waypoint deletions
- **Graceful degradation**: Handles edge cases (empty paths, goal reached, etc.)

### 4. **Comprehensive Data Logging**
Automatically generates CSV files for analysis:

**`trajectory_execution.csv`**:
- Timestamped robot positions (x, y, θ)
- Velocity commands (linear, angular)
- Curvature values (current, upcoming)
- 20 Hz logging frequency

**`waypoints_and_paths.csv`**:
- Original waypoints per pattern
- Smoothed path coordinates
- Pattern identification
- Sequence tracking

### 5. **Pattern-Based Testing**
Five geometric patterns for comprehensive validation:

| Pattern | Purpose | Validates |
|---------|---------|-----------|
| **Circle** | Constant curvature | Uniform circular motion, steady velocity |
| **Square** | Sharp corners | Corner handling, deceleration before turns |
| **Zigzag** | Rapid direction changes | Acceleration cycles, responsiveness |
| **Figure-Eight** | Variable curvature | Complex geometry, direction reversal |
| **Spiral** | Expanding radius | Continuously changing curvature, velocity scaling |

---

## 🛠️ Installation

### Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **Gazebo 11**
- **Python 3.8+**

### Required ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    python3-scipy
```

### Clone and Build

```bash
# Create workspace (if not exists)
mkdir -p ~/task_ws/src
cd ~/task_ws/src

# Clone the repository
git clone https://github.com/Brijrajsinh01/my_bot.git

# Build the package
cd ~/task_ws
colcon build --packages-select my_bot

# Source the workspace
source install/setup.bash
```

---

## 🎮 Usage

### Quick Start (Recommended)

**Step 1: Launch the simulation environment**

This starts Gazebo, RViz, and all necessary nodes (waypoint generator, path smoother):

```bash
cd ~/task_ws
source install/setup.bash
ros2 launch my_bot launch_sim.launch.py
```

**What launches:**
- ✅ Gazebo simulation with robot
- ✅ RViz2 visualization
- ✅ Waypoint generator (5 patterns)
- ✅ Path smoother (cubic splines)
- ✅ TF coordinate monitor

**Step 2: Start path execution (when ready)**

In a **new terminal**, launch the path executor separately:

```bash
cd ~/task_ws
source install/setup.bash
ros2 run my_bot path_executor.py
```

> **💡 Why separate?** This allows you to:
> - Visualize the generated waypoints and smooth paths first
> - Verify the path looks correct in RViz
> - Start execution only when you're ready
> - Restart execution without relaunching the entire system

---

## 🎬 Execution Flow

### Pattern Sequence

The system automatically cycles through 5 patterns:

```
1. Circle (40 waypoints)
   ↓ (completes)
2. Square (8 waypoints)
   ↓ (completes)
3. Zigzag (15 waypoints)
   ↓ (completes)
4. Figure-Eight (60 waypoints)
   ↓ (completes)
5. Spiral (100 waypoints)
   ↓ (completes)
```

Each pattern executes until the goal is reached, then automatically transitions to the next.

### What You'll See

**Terminal Output:**
```
🚀 Path Executor Started - OPTIMIZED VERSION
================================================
Algorithm: Pure Pursuit with Advanced Velocity Control
Look-ahead distance: 0.3 m
Control frequency: 20 Hz
Velocity range: 0.08-0.70 m/s
📊 Logging to: ~/task_ws/src/my_bot/config/trajectory_execution.csv
================================================

Progress: 25% | Vel: 0.45 m/s (target: 0.70) | Ang: 0.82 rad/s | Curv: 0.234 | Upcoming: 0.189
Progress: 50% | Vel: 0.68 m/s (target: 0.70) | Ang: 0.15 rad/s | Curv: 0.089 | Upcoming: 0.102
Progress: 75% | Vel: 0.32 m/s (target: 0.30) | Ang: 1.45 rad/s | Curv: 0.892 | Upcoming: 1.023
🎯 Goal reached in 45.32 seconds!
```

**RViz Visualization:**
- 🟢 Green markers: Waypoints
- 🔵 Blue line: Smooth path
- 🔴 Red arrow: Robot position and orientation
- 📍 Path following in real-time

---

## 🧪 Testing Error Recovery

### Demonstration: Real-time Path Recalibration

**Try this to see the adaptive error handling in action:**

1. **Start the system** as described above
2. **While the robot is moving**, open Gazebo
3. **Use the "Translate" tool** (T key) to move the robot to a different location
4. **Release the robot** and observe:
   - ✅ Path executor detects position change
   - ✅ Automatically recalculates closest point on path
   - ✅ Resumes execution without stopping
   - ✅ Maintains velocity (no reset to zero)

**What's happening under the hood:**
```python
# Intelligent pattern change detection
if is_pattern_changed():
    # New pattern detected → Reset velocity
    self.current_velocity = 0.0
else:
    # Path update → Maintain velocity and find closest point
    self.current_index = self.find_closest_path_index()
    # Continue smoothly!
```

This demonstrates:
- **Robustness**: Handles unexpected disturbances
- **Adaptability**: Recalibrates in real-time
- **Efficiency**: Doesn't restart from zero unnecessarily

---

## 📊 Data Analysis

### Generated CSV Files

After execution, analyze the logged data:

```bash
cd ~/task_ws/src/my_bot/config

# View trajectory data
head trajectory_execution.csv

# View waypoints and paths
head waypoints_and_paths.csv
```

## 🏗️ Architecture

### Node Communication

```
┌─────────────────┐
│ Waypoint        │
│ Generator       │──┐
└─────────────────┘  │
                     │ /waypoints (PoseArray)
                     ▼
                ┌─────────────────┐
                │ Path Smoother   │
                │ (Cubic Splines) │
                └─────────────────┘
                     │ /smooth_path (Path)
                     ▼
                ┌─────────────────┐
                │ Path Executor   │◄──── /odom (Odometry)
                │ (Pure Pursuit)  │
                └─────────────────┘
                     │ /cmd_vel (Twist)
                     ▼
                ┌─────────────────┐
                │ Gazebo Robot    │
                └─────────────────┘
```

### Algorithm Pipeline

1. **Waypoint Generation**: Creates geometric patterns (circle, square, etc.)
2. **Path Smoothing**: Applies cubic spline interpolation for smooth curves
3. **Velocity Optimization**: Calculates safe speeds based on path curvature
4. **Pure Pursuit Control**: Tracks the path using look-ahead algorithm
5. **Velocity Smoothing**: Applies acceleration limits for realistic motion
6. **Command Execution**: Publishes velocity commands to robot

---

## 🎓 Educational Value

This project demonstrates:

### Path Planning Concepts
- Cubic spline interpolation
- Waypoint-to-trajectory conversion
- Path smoothing techniques

### Control Theory
- Pure Pursuit algorithm
- Look-ahead distance tuning
- Velocity profile optimization

### Robotics Best Practices
- Modular node architecture
- Real-time error recovery
- Comprehensive logging
- Parameter-based configuration

### Software Engineering
- Clean code structure
- Separation of concerns
- Testability (5 pattern test suite)
- Documentation

---

## 🐛 Troubleshooting

### Robot doesn't move
```bash
# Check if path executor is running
ros2 node list | grep path_executor

# Verify topics are publishing
ros2 topic echo /smooth_path
ros2 topic echo /cmd_vel
```

### CSV files not created
```bash
# Check directory exists
ls -la ~/task_ws/src/my_bot/config/

# Verify node permissions
chmod +x ~/task_ws/src/my_bot/scripts/*.py
```

### Gazebo warnings/errors
These are harmless but annoying. To suppress:
- Set `verbose: 'false'` in `launch_sim.launch.py`
- Redirect gzclient output to log files

### Path looks jagged
- Increase `num_samples` in path smoother (default: 500)
- Reduce `velocity_smoothing_factor` for more responsive control

---

## 🤝 Contributing

Contributions are welcome! Feel free to:
- Report bugs
- Suggest features
- Submit pull requests
- Ask questions (even silly ones - we'll solve them together!)

Go ahead, clone it and have fun!

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

---

## 🙏 Acknowledgments

- **ROS 2 Community** for excellent documentation and tools
- **Gazebo** for realistic simulation environment
- **SciPy** for cubic spline interpolation

---

**Happy Navigating! 🚀🤖**

*Built with ❤️ using ROS 2 Humble*
