# Vegam Solution Assignment: Autonomous Container Entrance Detection

This repository contains a ROS 2 Humble package designed for a differential drive robot to detect and navigate towards a container entrance using 2D LiDAR data. The project is implemented in C++ and optimized for simulation in NVIDIA Isaac Sim.

## 🚀 Overview

The system identifies container entrances by detecting depth discontinuities (corners) in LiDAR scan data. Once an entrance is confirmed with high confidence, the bot aligns itself and positions itself at a fixed distance from the entrance vertices.

### Key Features
* **Thread-Safe Processing:** LiDAR data is processed in a dedicated worker thread using `std::condition_variable` and `std::mutex` to ensure zero lag in the ROS 2 executor.
* **Feature Extraction:** Implements a custom algorithm to detect corners and entrance midpoints using depth "jumps."
* **Parameter-Driven:** Control speeds, detection thresholds, and confidence levels via `.yaml` configuration.
* **Robust Detection:** Uses a confidence-based scoring system and reset counters to filter out LiDAR noise and transient obstacles.

---

## 🛠️ System Architecture

The project is split into two core classes:
1.  **`ContainerBot`**: Handles the hardware abstraction (or simulation API) for movement using `geometry_msgs/Twist`.
2.  **`Detection`**: Manages the LiDAR subscription, threading, and the core entrance-finding logic.

---

## 📐 Detection Logic

The entrance is identified through the following steps:
1.  **Discontinuity Search:** Scans the `ranges` array for Euclidean distance jumps $> 1.0m$.
2.  **Corner Identification:** Marks the specific indices where walls end and the container interior begins.
3.  **Entrance Verification:** Validates that the space between two corners is deeper than the corners themselves (ensuring it's an opening, not a pillar).
4.  **Confidence Scoring:** Requires 20 consecutive successful detections before triggering the "Entrance Detected" state.

---

## ⚙️ Installation & Usage

### Prerequisites
* Ubuntu 22.04
* ROS 2 Humble
* NVIDIA Isaac Sim (or a compatible 2D LiDAR simulator)

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select vegam_solution_assignment
source install/setup.bash
