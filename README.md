# Jackal Control Humble - ROS2 Package

## Overview

This package provides an enhanced control interface for the Jackal robot using ROS 2. It includes the following key features:

- **Twist Multiplexer (Twist Mux)**: A multiplexer node to handle multiple velocity inputs, selecting the appropriate control command for the robot.
- **Odometry Node**: An odometry node that computes the robot's pose and velocity using encoder data.
- **MCU Nodes via micro-ROS**: A micro-ROS agent that facilitates communication with the robot's microcontroller (MCU) nodes, providing additional sensor and control data.

## Features

1. **Twist Mux for Velocity Control**:
   - Multiplexes velocity commands from various sources (joystick, interactive_markers, teleoperation, cmd_vel) and ensures that only one input is active at a time based on priority.
   
2. **Odometry Computation**:
   - Processes encoder data from the robot's wheels to estimate the robot's position and velocity in the world frame.
   - Publishes odometry information using the standard ROS 2 `nav_msgs/Odometry` message type.

3. **micro-ROS MCU Nodes**:
   - Utilizes the micro-ROS agent to enable communication with the robot's microcontroller units (MCUs).
   - Supports low-level hardware access and control, such as sensor data reading and motor control.
     
## Services

Includes the boot service to enable all nodes and allows ps4 control teleoperation by only turning the robot ON.
