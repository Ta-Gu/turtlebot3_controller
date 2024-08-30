# Turtlebot3 Controller to avoid obstacles

This package provides advanced control scripts for the Turtlebot3 robot in a Gazebo simulation environment using ROS2. The primary script allows the robot to move in a square pattern and bypass obstacles when detected.

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
