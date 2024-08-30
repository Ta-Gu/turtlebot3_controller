# Turtlebot3 Controller to avoid obstacles

This package provides an obstacle avoidance script for the Turtlebot3 robot in a Gazebo simulation environment using ROS2. The primary script allows the robot to move in a square pattern and bypass obstacles when detected.

## Package Contents

- **`advanced_square_movement.py`**: An example script to control the Turtlebot3 to move in a square pattern while avoiding obstacles.
- **`setup.py`**: A setup script for building the package.
- **`package.xml`**: The package configuration file.

## Prerequisites

- **ROS2 Humble**: Make sure you have ROS2 Humble installed on your system.
- **Turtlebot3 Packages**: You need to have Turtlebot3 simulation packages installed.
- **Gazebo**: Gazebo should be installed for running the robot simulation.

## Installation

### 1. Set Up Your Workspace

First, create a ROS2 workspace if you don't have one:

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/
colcon build
source install/setup.bash
```

### 2. Clone the Package

Navigate to the src directory of your workspace and clone this repository:

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/Ta-Gu/turtlebot3_controller.git
```

### 3. Install Dependencies

Install any dependencies specified in the package.xml:

```bash
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Source the Workspace

Source the workspace to overlay this package on your ROS2 environment:

```bash
source install/setup.bash
```

## Usage

### 1. Launch the Gazebo Simulation

Start the Turtlebot3 simulation in Gazebo with the following command:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Run the Advanced Movement Script

Open a new terminal, source your workspace, and run the advanced movement script:

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_controller advanced_square_movement
```

### 3. Testing and Verifying

You can monitor the robot's movement and behavior using Gazebo and RViz. To open RViz:

```bash
ros2 run rviz2 rviz2
```
