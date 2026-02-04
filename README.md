# Glass Detection & Auto-Door Navigation System

Robust Autonomous Navigation in Glass-Walled Environments using LiDAR-Sonar Sensor Fusion.

[Status: Patented Technology]

## Overview
This ROS package provides a solution for mobile robots to detect transparent obstacles (glass) and navigate through automatic doors efficiently.
By fusing LiDAR with Ultrasonic Sensors (Sonar), the system overcomes the transparency problem of standard laser scanners. Additionally, it implements a unique navigation logic to resolve the "closed door dilemma," allowing the robot to wait for and pass through automatic doors without path-planning errors.

Note: The glass detection method implemented in this project utilizes patented technology.

## Core Technology (Patented)

### Vertical Reflection Extraction Logic
Standard 2D LiDAR pulses usually pass through glass. However, we focused on the physical property that "weak reflections occur only when the laser hits the glass perpendicularly."

Since these weak reflections are indistinguishable from noise, this system uses a Sensor Fusion approach:

1. Coarse Detection: Sonar detects the approximate presence of an object (glass).
2. Fine Extraction: The system filters LiDAR points (sensor_fusion.py). It extracts only the "weak LiDAR points" that exist within the Sonar's detection range.

Result: This allows for high-resolution mapping of glass walls, which is impossible with Sonar alone, while eliminating the false negatives common with LiDAR alone.

## Navigation Logic: The "Auto-Door Dilemma"

Navigating through glass automatic doors presents a specific dilemma:

- The Problem: If we map the closed door as a "wall," the global planner treats it as a permanent obstacle and cannot generate a path through it. If we don't map it, the robot crashes into the door before it opens.
- The Solution: We implemented a two-part strategy.

### 1. Planner Separation
- Global Planner: Uses the static map to generate a path assuming the door is passable.
- Local Planner: Ignores the static map for immediate collision avoidance. Instead, it relies on real-time sensor data and our Speed Controller to safely approach and wait for the door to open.

### 2. Persistent Footprint Clearing (Map Update Logic)
To ensure smooth re-entry or return trips:

- Logic: Any area the robot has physically traversed (its footprint) is marked as "Permanently Clear" in the costmap.
- Benefit: Even if the automatic door closes behind the robot, the map retains that area as a "valid path." When the robot needs to return, the Global Planner can immediately generate a path through the closed door (knowing it was previously passable), allowing the robot to approach it directly instead of searching for a detour.

## Safety Features

### Adaptive Speed Controller (speed_controller.py)
Ensures safe interaction with doors and glass walls.

- Dynamic Velocity: Automatically scales the robot's max velocity (Normal -> Slow -> Crawl -> Stop) based on real-time Sonar distance.
- Noise Filtering: Implements a time-series history buffer (approx. 0.25s / 5 frames) to filter out sporadic sensor noise, preventing sudden stops due to ghost readings.

## Tech Stack
- Framework: ROS 1 (Noetic)
- Languages: Python 3
- Hardware: 2D LiDAR, Ultrasonic Sensors (x3), Mobile Robot Base
- Key Libraries: rospy, numpy, scipy.spatial (KDTree), tf
